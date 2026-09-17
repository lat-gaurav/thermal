#!/usr/bin/env python3
"""Broad-strokes health check for thermal-live.service (mota mota, not per-frame).

This does NOT replace deploy/run_live.sh's own device/exclusivity checks --
those decide whether the pipeline can start at all. This tool answers the next
question: given that systemd says the unit is "active", is it actually doing
anything useful right now, or sitting there stuck/crash-looping/overheating?

Five checks, each independent so one failure doesn't hide another:

  1. systemd unit state       -- active / crash-looping (auto-restart) / failed
  2. journal freshness        -- a recent [status] line, and no FATAL/traceback
                                  since the current process started
  3. device nodes             -- camera (+ Cube UART in track/flight mode),
                                  read from /etc/default/thermal-live
  4. thermal throttling       -- CPU temp and vcgencmd's throttle word
  5. recording disk space     -- log dir free space vs the configured reserve

Exit code is the worst severity seen: 0 OK, 1 WARN, 2 CRIT -- so this drops
straight into a cron job, a systemd OnFailure= unit, or a monitoring check.

    tools/health_check.py                  human-readable report to stdout
    tools/health_check.py --json           same checks, one JSON object
    tools/health_check.py --quiet          print nothing on OK, only WARN/CRIT
    tools/health_check.py --unit foo.service --env /etc/default/foo
"""
import argparse
import datetime
import json
import os
import re
import subprocess
import sys
import time

OK, WARN, CRIT = 0, 1, 2
_SEVNAME = {OK: "OK", WARN: "WARN", CRIT: "CRIT"}

DEFAULT_UNIT = "thermal-live.service"
DEFAULT_ENV_FILE = "/etc/default/thermal-live"

# Fallbacks if config.py or the env file can't be read, so this script still
# runs standalone (e.g. copied onto another box for a quick check).
DEFAULT_STATUS_EVERY_S = 60.0
DEFAULT_THROTTLE_TEMP_C = 80.0  # the Pi 5 soft-throttles near here (see config.py)


def _run(cmd, timeout=10):
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                               timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError) as e:
        return "__ERROR__:%s" % e


def parse_env_file(path):
    """KEY=VALUE lines from an EnvironmentFile= -- comments and blanks skipped."""
    env = {}
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, _, v = line.partition("=")
                env[k.strip()] = v.strip()
    except OSError:
        pass
    return env


def check_unit(unit):
    """Is the service actually running, or crash-looping / failed?"""
    out = _run(["systemctl", "show", unit, "-p",
                "ActiveState,SubState,Result,NRestarts,ExecMainStartTimestamp,"
                "ExecMainPID"])
    fields = dict(l.split("=", 1) for l in out.splitlines() if "=" in l)
    active = fields.get("ActiveState", "?")
    sub = fields.get("SubState", "?")
    result = fields.get("Result", "?")
    try:
        n_restarts = int(fields.get("NRestarts", "0"))
    except ValueError:
        n_restarts = 0
    pid = fields.get("ExecMainPID", "0")
    started = fields.get("ExecMainStartTimestamp", "")

    detail = "%s/%s pid=%s restarts=%d result=%s" % (
        active, sub, pid, n_restarts, result)

    if active == "failed" or sub == "failed":
        return CRIT, "unit has FAILED (%s)" % detail, fields
    if sub == "auto-restart":
        return CRIT, "crash-looping -- systemd is retrying ExecStart (%s)" % detail, fields
    if active != "active":
        return WARN, "unit is %s, not active (%s)" % (active, detail), fields
    if pid == "0":
        return WARN, "active but no main PID -- likely mid-restart (%s)" % detail, fields
    return OK, "running (%s, since %s)" % (detail, started or "?"), fields


_FATAL_RE = re.compile(r"\bFATAL\b|\bTraceback\b|\berror\b", re.IGNORECASE)
_STATUS_RE = re.compile(r"^\S+ \S+ \S+ \[status\]")


def check_journal(unit, since_ts, status_every_s):
    """Recent activity: a fresh [status] line, and nothing FATAL since start."""
    out = _run(["journalctl", "-u", unit, "-n", "500", "--no-pager",
                "-o", "short-iso", "--since", since_ts] if since_ts else
               ["journalctl", "-u", unit, "-n", "500", "--no-pager", "-o", "short-iso"])
    if out.startswith("__ERROR__"):
        return WARN, "could not read journal (%s) -- is this user in the adm/systemd-journal group?" % out
    lines = out.splitlines()
    if not lines:
        return WARN, "no journal entries for %s yet" % unit

    last_status_age = None
    fatal_line = None
    for line in lines:
        m = re.match(r"^(\S+ \S+)", line)
        ts = m.group(1) if m else None
        if "[status]" in line and ts:
            try:
                # -o short-iso: "2026-09-17T12:27:20+0530" -- %z parses the
                # numeric (colon-less) offset directly, no string surgery needed.
                t = datetime.datetime.strptime(ts, "%Y-%m-%dT%H:%M:%S%z")
                last_status_age = (datetime.datetime.now(datetime.timezone.utc)
                                    - t).total_seconds()
            except ValueError:
                pass
        if _FATAL_RE.search(line) and "[run_live]   " not in line:
            fatal_line = line.strip()[-160:]

    parts = []
    sev = OK
    if fatal_line:
        sev = CRIT
        parts.append("saw an error/FATAL line recently: %s" % fatal_line)
    if last_status_age is None:
        # Not necessarily bad -- e.g. web/fb mode print no [status], or it just
        # (re)started and hasn't hit the first interval yet.
        parts.append("no [status] line seen in the last 500 journal lines")
    else:
        max_age = status_every_s * 3
        if last_status_age > max_age:
            sev = max(sev, WARN)
            parts.append("last [status] was %.0fs ago (expected every ~%.0fs)"
                         % (last_status_age, status_every_s))
        else:
            parts.append("last [status] %.0fs ago" % last_status_age)
    return sev, "; ".join(parts)


def check_devices(env):
    mode = env.get("THERMAL_MODE", "web")
    camera = env.get("THERMAL_DEVICE", "/dev/thermal0")
    cube = env.get("THERMAL_CUBE_DEVICE", "/dev/ttyAMA0")

    missing = []
    if not os.path.exists(camera):
        missing.append(camera)
    if mode in ("track", "flight") and not os.path.exists(cube):
        missing.append(cube)

    if missing:
        return CRIT, "missing device node(s) for mode=%s: %s" % (mode, ", ".join(missing))
    return OK, "mode=%s, present: %s" % (mode, camera + ((" " + cube) if mode in ("track", "flight") else ""))


def check_thermal(throttle_temp_c):
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as fh:
            temp_c = int(fh.read()) / 1000.0
    except (OSError, ValueError):
        temp_c = float("nan")

    out = _run(["vcgencmd", "get_throttled"]).strip()
    word = out.split("=")[-1] if "=" in out else "?"
    try:
        bits = int(word, 16)
    except ValueError:
        bits = 0

    # vcgencmd get_throttled bit layout: bits 0-3 = happening now (under-voltage,
    # freq capped, throttled, soft temp limit); bits 16-19 = happened since boot.
    # Only "happening now" (low nibble) is actionable right this second.
    now_bits = bits & 0xF
    detail = "%.1fC throttled=%s" % (temp_c, word)
    if now_bits:
        return WARN, "throttling ACTIVE right now (%s)" % detail
    if temp_c >= throttle_temp_c:
        return WARN, "CPU temp near the throttle point (%s)" % detail
    return OK, detail


def check_disk(env):
    log_dir = env.get("THERMAL_LOG_DIR", "/home/rpi5/thermal/logs")
    fallback = env.get("THERMAL_LOG_DIR_FALLBACK", "/home/rpi5/flight-logs")
    reserve_mb = int(env.get("THERMAL_RAW_RESERVE_MB", "2048") or "2048")
    reserve_fallback_mb = int(env.get("THERMAL_RAW_RESERVE_MB_FALLBACK", "8192") or "8192")
    require_external = env.get("THERMAL_REQUIRE_EXTERNAL", "1") == "1"

    def fs_of(p):
        while not os.path.exists(p) and p != "/":
            p = os.path.dirname(p) or "/"
        return p

    def statvfs_mb(p):
        st = os.statvfs(fs_of(p))
        return st.f_bavail * st.f_frsize / 1024.0 / 1024.0

    root_dev = os.stat(fs_of("/")).st_dev
    log_dev = os.stat(fs_of(log_dir)).st_dev if os.path.exists(fs_of(log_dir)) else None
    on_root = (log_dev == root_dev)

    if require_external and on_root:
        path, reserve = fallback, reserve_fallback_mb
        note = "SSD not mounted -- would fall back to %s" % fallback
    else:
        path, reserve = log_dir, reserve_mb
        note = None

    free_mb = statvfs_mb(path)
    usable_mb = free_mb - reserve
    detail = "%s: %.0f MB free (%.0f MB reserved -> %.0f MB usable)" % (
        path, free_mb, reserve, usable_mb)
    if note:
        detail = note + "; " + detail

    if usable_mb <= 0:
        return WARN, "recording will REFUSE, out of budget: %s" % detail
    if usable_mb < reserve:  # less usable than the reserve itself -- getting close
        return WARN, "recording budget is thin: %s" % detail
    return OK, detail


def load_status_every_s():
    try:
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        import config
        return float(getattr(config, "FLIGHT_STATUS_EVERY_S", DEFAULT_STATUS_EVERY_S))
    except Exception:
        return DEFAULT_STATUS_EVERY_S


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unit", default=DEFAULT_UNIT,
                     help="systemd unit to check (default %(default)s)")
    ap.add_argument("--env", default=DEFAULT_ENV_FILE,
                     help="EnvironmentFile to read mode/device/disk settings from")
    ap.add_argument("--throttle-temp-c", type=float, default=DEFAULT_THROTTLE_TEMP_C)
    ap.add_argument("--json", action="store_true", help="emit one JSON object instead")
    ap.add_argument("--quiet", action="store_true",
                     help="print nothing when everything is OK (still sets exit code)")
    args = ap.parse_args()

    env = parse_env_file(args.env)
    status_every_s = load_status_every_s()

    unit_sev, unit_msg, unit_fields = check_unit(args.unit)
    since = unit_fields.get("ExecMainStartTimestamp", "")
    journal_sev, journal_msg = check_journal(args.unit, since, status_every_s)
    device_sev, device_msg = check_devices(env)
    thermal_sev, thermal_msg = check_thermal(args.throttle_temp_c)
    disk_sev, disk_msg = check_disk(env)

    checks = [
        ("unit", unit_sev, unit_msg),
        ("journal", journal_sev, journal_msg),
        ("devices", device_sev, device_msg),
        ("thermal", thermal_sev, thermal_msg),
        ("disk", disk_sev, disk_msg),
    ]
    worst = max(sev for _, sev, _ in checks)

    if args.json:
        print(json.dumps({
            "unit": args.unit,
            "overall": _SEVNAME[worst],
            "checks": {name: {"severity": _SEVNAME[sev], "detail": msg}
                       for name, sev, msg in checks},
            "ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }, indent=2))
    else:
        if not (args.quiet and worst == OK):
            print("=" * 72)
            print("health check: %s -- overall %s" % (args.unit, _SEVNAME[worst]))
            for name, sev, msg in checks:
                print("  [%-4s] %-8s %s" % (_SEVNAME[sev], name, msg))
            print("=" * 72)

    sys.exit(worst)


if __name__ == "__main__":
    main()
