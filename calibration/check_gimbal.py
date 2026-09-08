#!/usr/bin/env python3
"""Is gimbal telemetry arriving from the USB-TTL converter? One command, one verdict.

    check_gimbal.py                 listen 3 s and report
    check_gimbal.py -t 30           listen longer
    check_gimbal.py --watch         keep checking until Ctrl-C (catches dropouts)
    check_gimbal.py --json          machine-readable
    check_gimbal.py --raw           dump the first bytes seen, for protocol debugging

RECEIVE ONLY. Nothing is ever transmitted to the gimbal.

Checks the whole chain and says which link is broken, rather than just "no data":

    1. is the FTDI converter on the USB bus at all
    2. did a tty node and the /dev/local_dds symlink get created
    3. can the port be opened, or does something else hold it
    4. are raw bytes arriving
    5. do those bytes parse as the gimbal's A5 5A frames with a valid CRC
    6. are yaw/pitch actually changing, or frozen at one value

Steps 4 and 5 are kept separate on purpose. "Bytes arriving but nothing parses"
means the link is alive at the wrong baud or is speaking a different protocol,
which is a completely different problem from "no bytes at all" (gimbal unpowered,
or its TX line not connected). Reporting them as one failure hides that.

WIRE FORMAT: A5 5A | 8 x float32 LE | CRC16 (MCRF4XX over bytes[2..33]) = 36 B,
~1010 Hz at 921600 8N1. float[0] = yaw, float[1] = pitch, degrees.

WHAT IS NOT MEASURED: per-frame arrival jitter. Python's buffered reads batch
several frames per read() call and stamp them together, so any jitter figure from
here would be an artifact of this script rather than a property of the link. The
frame RATE is count over elapsed time and is unaffected. If you ever need real
per-frame arrival timing, it has to be done in C with VMIN=0/VTIME=0 and poll();
see tlm_log.c in git history (commit a6855b0).

Exit status: 0 = telemetry healthy, 1 = something is wrong (see the verdict line).
"""
import argparse
import glob
import json
import os
import struct
import subprocess
import sys
import time

FTDI_VIDPID = "0403:6001"
PORT_CANDIDATES = [
    "/dev/local_dds",                                              # project udev rule
    "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*-if00-port0",      # stock udev, by serial
    "/dev/ttyUSB0",                                                 # last resort
]
TLM_LEN = 36
DEF_BAUD = 921600
ALT_BAUD = 115200          # gimbal_link.py used this historically
EXPECT_HZ = 1010.0


def crc16(d):
    """CRC-16/MCRF4XX, init 0xFFFF -- matches the gimbal firmware."""
    crc = 0xFFFF
    for b in d:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def usb_present():
    try:
        out = subprocess.run(["lsusb"], capture_output=True, text=True,
                             timeout=10).stdout
    except Exception:                                   # noqa: BLE001
        return None, "lsusb unavailable"
    for line in out.splitlines():
        if FTDI_VIDPID in line:
            return True, line.strip()
    return False, None


def find_port(explicit=None):
    if explicit:
        if os.path.exists(explicit):
            return explicit, None
        return None, "EXPLICIT:%s does not exist" % explicit
    for pat in PORT_CANDIDATES:
        for p in sorted(glob.glob(pat)):
            if os.path.exists(p):
                return p, None
    return None, "none of %s exist" % ", ".join(PORT_CANDIDATES)


def port_holders():
    """Processes we can see holding a ttyUSB. Root-owned ones stay invisible."""
    out = []
    for d in glob.glob("/proc/[0-9]*"):
        try:
            for fd in os.listdir(d + "/fd"):
                tgt = os.readlink("%s/fd/%s" % (d, fd))
                if "ttyUSB" in tgt:
                    cmd = open(d + "/cmdline", "rb").read().replace(b"\0", b" ")
                    # cmdlines can contain newlines (python3 -c "..."), which would
                    # break the one-hint-per-line report; collapse all whitespace.
                    flat = " ".join(cmd.decode("utf-8", "replace").split())
                    out.append("pid %s: %s" % (os.path.basename(d), flat[:70]))
                    break
        except Exception:                               # noqa: BLE001
            continue
    return out


def listen(port, baud, secs, want_raw=False):
    """Read for `secs`, parse frames, never write. Returns a stats dict."""
    import serial
    st = {"baud": baud, "bytes": 0, "frames": 0, "crc_errors": 0, "resync": 0,
          "elapsed": 0.0, "raw_head": None, "open_error": None,
          "yaw": None, "pitch": None, "yaw_span": None, "pitch_span": None,
          "identical_frames": False, "floats_last": None, "read_error": None}
    try:
        ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1,
                            timeout=0.05)
    except Exception as e:                              # noqa: BLE001
        st["open_error"] = "%s" % e
        return st
    try:
        ser.reset_input_buffer()
        buf = bytearray()
        t0 = time.monotonic()
        ymin = ymax = pmin = pmax = None
        seen = set()
        while time.monotonic() - t0 < secs:
            try:
                chunk = ser.read(4096)
            except Exception as e:                      # noqa: BLE001
                st["read_error"] = "%s" % e
                break
            if not chunk:
                continue
            st["bytes"] += len(chunk)
            if want_raw and st["raw_head"] is None:
                st["raw_head"] = " ".join("%02X" % b for b in chunk[:48])
            buf += chunk
            i = 0
            while len(buf) - i >= TLM_LEN:
                if buf[i] != 0xA5 or buf[i + 1] != 0x5A:
                    i += 1
                    st["resync"] += 1
                    continue
                frame = bytes(buf[i:i + TLM_LEN])
                if crc16(frame[2:34]) != struct.unpack("<H", frame[34:36])[0]:
                    st["crc_errors"] += 1
                    i += 1                  # a real frame can start 1 byte in
                    continue
                vals = struct.unpack("<8f", frame[2:34])
                st["frames"] += 1
                st["floats_last"] = [round(v, 4) for v in vals]
                y, p = vals[0], vals[1]
                if ymin is None:
                    ymin = ymax = y
                    pmin = pmax = p
                ymin, ymax = min(ymin, y), max(ymax, y)
                pmin, pmax = min(pmin, p), max(pmax, p)
                if len(seen) < 4:
                    seen.add(frame[2:34])
                i += TLM_LEN
            del buf[:i]
        st["elapsed"] = time.monotonic() - t0
    finally:
        ser.close()
    if st["frames"]:
        st["yaw"] = round(ymax, 4)
        st["pitch"] = round(pmax, 4)
        st["yaw_span"] = round(ymax - ymin, 4)
        st["pitch_span"] = round(pmax - pmin, 4)
        st["identical_frames"] = st["frames"] > 20 and len(seen) == 1
    st["hz"] = round(st["frames"] / st["elapsed"], 1) if st["elapsed"] > 0 else 0.0
    st["kbps"] = round(st["bytes"] / st["elapsed"] / 1000, 1) if st["elapsed"] > 0 else 0.0
    return st


def diagnose(res):
    """Return (ok, verdict, [hints]) from the collected evidence."""
    if res["usb"] is False:
        return False, "USB-TTL converter is not on the bus", [
            "The FTDI adapter (%s) is not enumerated -- it is unplugged, the cable"
            " is dead, or the hub port has no power." % FTDI_VIDPID,
            "Check with: lsusb | grep 0403:6001",
        ]
    if res["port"] is None:
        err = res["port_error"] or ""
        if err.startswith("EXPLICIT:"):
            return False, "the port you gave does not exist", [
                err[len("EXPLICIT:"):],
                "Drop --port to let it auto-detect, or list what is there:"
                "  ls -l /dev/serial/by-id/",
            ]
        return False, "converter present but no tty node", [
            "The FTDI is on the bus but no /dev/ttyUSB* appeared: %s" % err,
            "Check the driver bound: lsmod | grep ftdi_sio",
            "/dev/local_dds comes from 99-drone-ports.rules -- confirm it is installed.",
        ]
    st = res["listen"]
    if st["open_error"]:
        hints = ["Could not open %s: %s" % (res["port"], st["open_error"])]
        if "Errno 16" in st["open_error"] or "busy" in st["open_error"].lower():
            hints.append("Something else holds the port. ModemManager probes serial"
                         " ports as modems -- check: journalctl -u ModemManager -n 20")
            hints += res["holders"] or [
                "No holder visible to this user; a root-owned process would not show."]
        elif "denied" in st["open_error"].lower():
            hints.append("Permission denied -- are you in the dialout group? (id -nG)")
        return False, "cannot open the port", hints
    if st["read_error"]:
        msg = st["read_error"]
        hints = ["Reading stopped partway: %s" % msg]
        if "multiple access" in msg or "returned no data" in msg:
            hints.append("pyserial names the two causes itself. Both are likely here:"
                         " another process draining the same port, or the converter"
                         " disconnecting mid-read.")
            hints += res["holders"] or [
                "No other reader visible to this user; a root-owned one would not"
                " show. Check: journalctl -u ModemManager -n 20"]
            hints.append("A tty permits several readers and splits the bytes between"
                         " them -- stop the other consumer and re-run.")
        else:
            hints.append("Check the converter is still on the bus: lsusb | grep 0403:6001")
        return False, "the read failed partway through", hints
    if st["bytes"] == 0:
        return False, "port opens but no bytes arrive", [
            "The link is up on the host side and completely silent.",
            "The gimbal is unpowered, or its TX line is not reaching the converter's RX.",
            "Check the gimbal has power and the TX/RX pair is not swapped.",
        ]
    if st["frames"] == 0:
        return False, "bytes arriving but nothing parses as a gimbal frame", [
            "Received %d bytes (%.1f kB/s) at %d baud, but found no valid"
            " A5 5A frame." % (st["bytes"], st["kbps"], st["baud"]),
            "That usually means the wrong baud rate, or a different device on this port.",
            "Re-run with --raw to see the bytes, and try --baud %d." % (
                ALT_BAUD if st["baud"] == DEF_BAUD else DEF_BAUD),
        ]
    hints = []
    ok = True
    # A tty allows several readers at once, and they split the byte stream between
    # them: each sees fragments, so resync climbs and CRCs start failing. Measured
    # here -- alone: crc 0, resync 7 per 3000 frames; with a second reader on the
    # same port: crc 4, resync 113. Without this check that reads as "healthy".
    if st["frames"] and st["resync"] > 50 + st["frames"] * 0.01:
        ok = False
        hints.append("%d resync bytes on %d frames is far above the ~35 expected from"
                     " one partial frame at start. Another process is very likely"
                     " reading the same port and stealing bytes."
                     % (st["resync"], st["frames"]))
        hints += res["holders"] or [
            "No other reader visible to this user; a root-owned one would not show."
            " Check: journalctl -u ModemManager -n 20"]
    if st["crc_errors"] > max(2, st["frames"] * 0.01):
        ok = False
        hints.append("%d CRC errors on %d frames (%.1f%%) -- noisy line, marginal"
                     " ground, or baud slightly off."
                     % (st["crc_errors"], st["frames"],
                        100.0 * st["crc_errors"] / st["frames"]))
    if st["hz"] < EXPECT_HZ * 0.8:
        ok = False
        hints.append("Rate is %.1f Hz, well below the expected ~%.0f Hz -- frames are"
                     " being lost or the gimbal is publishing slowly."
                     % (st["hz"], EXPECT_HZ))
    if st["identical_frames"]:
        ok = False
        hints.append("Every frame is byte-identical: the gimbal is transmitting but"
                     " its values are not updating. Stale data, not live telemetry.")
    elif st["yaw_span"] == 0.0 and st["pitch_span"] == 0.0:
        hints.append("yaw and pitch did not change at all. Plausible for a parked"
                     " gimbal, but confirm by moving it while this runs.")
    if ok:
        return True, "gimbal telemetry is healthy", hints
    return False, "telemetry is arriving but degraded", hints


def run_once(args):
    res = {"port": None, "port_error": None, "holders": [], "listen": None}
    res["usb"], res["usb_line"] = usb_present()
    res["port"], res["port_error"] = find_port(args.port)
    if res["port"]:
        res["holders"] = port_holders()
        res["listen"] = listen(res["port"], args.baud, args.seconds, args.raw)
        # Bytes but no frames is worth one automatic retry at the other rate this
        # project has used, rather than making the user guess. Pick the other rate
        # relative to whatever was actually tried -- keying off DEF_BAUD meant
        # `--baud 115200` never retried at all.
        if (not args.no_autobaud and res["listen"]["frames"] == 0
                and res["listen"]["bytes"] > 0):
            other = ALT_BAUD if args.baud != ALT_BAUD else DEF_BAUD
            alt = listen(res["port"], other, min(2.0, args.seconds), args.raw)
            res["listen_alt"] = alt
            if alt["frames"] > 0:
                res["listen"] = alt
    ok, verdict, hints = diagnose(res)
    res["ok"], res["verdict"], res["hints"] = ok, verdict, hints
    return res


def report(res, args):
    print("usb        %s" % ("FTDI present -- " + res["usb_line"] if res["usb"]
                             else "FTDI NOT on the bus (%s)" % FTDI_VIDPID))
    if res["port"]:
        tgt = os.path.realpath(res["port"])
        print("port       %s%s" % (res["port"],
                                   "" if tgt == res["port"] else " -> " + tgt))
    else:
        print("port       (none found)")
    st = res["listen"]
    if st and not st["open_error"]:
        print("listened   %.1f s at %d baud, receive only\n" % (st["elapsed"], st["baud"]))
        print("bytes      %d  (%.1f kB/s)" % (st["bytes"], st["kbps"]))
        print("frames     %d valid  (%.1f Hz)   crc errors %d   resync %d"
              % (st["frames"], st["hz"], st["crc_errors"], st["resync"]))
        if st["frames"]:
            print("yaw        %9.4f deg   span %.4f over %.1f s"
                  % (st["yaw"], st["yaw_span"], st["elapsed"]))
            print("pitch      %9.4f deg   span %.4f"
                  % (st["pitch"], st["pitch_span"]))
            if args.verbose and st["floats_last"]:
                print("floats     %s" % st["floats_last"])
        if st["raw_head"]:
            print("raw head   %s" % st["raw_head"])
        if res.get("listen_alt") and res["listen_alt"] is not st:
            print("note       also tried %d baud: %d frames"
                  % (ALT_BAUD, res["listen_alt"]["frames"]))
        print()
    print("%s  %s" % ("PASS" if res["ok"] else "FAIL", res["verdict"]))
    for h in res["hints"]:
        print("      - %s" % h)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-t", "--seconds", type=float, default=3.0,
                    help="how long to listen (default 3)")
    ap.add_argument("--port", help="override the auto-detected port")
    ap.add_argument("--baud", type=int, default=DEF_BAUD,
                    help="baud rate (default %d)" % DEF_BAUD)
    ap.add_argument("--no-autobaud", action="store_true",
                    help="do not retry at %d when nothing parses" % ALT_BAUD)
    ap.add_argument("--watch", action="store_true",
                    help="repeat until Ctrl-C, one line per check -- use this to"
                         " catch intermittent dropouts")
    ap.add_argument("--json", action="store_true", help="emit JSON")
    ap.add_argument("--raw", action="store_true",
                    help="show the first bytes received, for protocol debugging")
    ap.add_argument("--verbose", action="store_true",
                    help="also print all 8 floats of the last frame")
    args = ap.parse_args()

    try:
        import serial                                   # noqa: F401
    except ImportError:
        print("pyserial not installed:  pip3 install pyserial", file=sys.stderr)
        return 2

    if args.watch:
        print("watching -- Ctrl-C to stop")
        bad = 0
        try:
            while True:
                r = run_once(args)
                st = r["listen"] or {}
                stamp = time.strftime("%H:%M:%S")
                if r["ok"]:
                    print("  %s  OK    %.1f Hz  crc %d  yaw %.3f  pitch %.3f"
                          % (stamp, st.get("hz", 0), st.get("crc_errors", 0),
                             st.get("yaw") or 0.0, st.get("pitch") or 0.0))
                else:
                    bad += 1
                    print("  %s  FAIL  %s" % (stamp, r["verdict"]))
        except KeyboardInterrupt:
            print("\nstopped -- %d failed check(s)" % bad)
            return 1 if bad else 0

    res = run_once(args)
    if args.json:
        print(json.dumps(res, indent=2))
    else:
        report(res, args)
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
