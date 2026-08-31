#!/usr/bin/env python3
"""Set the Artosyn "Sirius" thermal core's frame rate over its USB serial port.

Jetson-portable variant of set_fps.py. The serial half of set_fps.py works here
unchanged; the only thing that did not port is the video node it times frames on.
set_fps.py hardcodes VIDEO_DEV = "/dev/thermal0", which is created by the RPi's
custom 99-thermal-cam.rules. That rule is not installed on this Jetson, so every
--show/--measure died with "cannot measure: /dev/thermal0 does not exist" while
the camera was in fact present and streaming.

Rather than require the custom rule, this resolves the capture node through the
symlink stock udev already makes (60-persistent-v4l.rules):

    /dev/v4l/by-id/usb-Artosyn_Sirius_<serial>-video-index0

That keeps set_fps.py's "pin by USB serial, never by videoN" rule -- the numbers
drift across re-enumeration -- without depending on a hand-installed rule file.
/dev/thermal0 is still preferred when it exists, so this file also runs on the RPi.

Picking the right node matters: the core exposes TWO v4l2 nodes off one serial.
On this Jetson video0 is the capture node (ID_V4L_CAPABILITIES=:capture:) and
video1 is a metadata node (:), which enumerates no capture formats and would
fail to stream. index0/index1 in the by-id names follow that same order.

    set_fps_jetson.py 25                 switch the core to 25 fps
    set_fps_jetson.py 50                 switch it back to 50 fps (power-on default)
    set_fps_jetson.py 25 --measure       set it, then time the frames delivered
    set_fps_jetson.py --show             don't change anything; just time the rate
    set_fps_jetson.py --list             show the core's serial/video/tty nodes and exit

The core accepts only these two rates: control code 0x05 = 25 fps, 0x06 = 50 fps.
There is NO read-back command for frame rate, so --show/--measure time delivered
frames instead; that is the only way to know what the core is doing.

WHAT 50 fps ACTUALLY GIVES YOU OVER USB: about 30, not 50. 1280x1024 GREY at 50 fps
is 65.5 MB/s = 524 Mbps, above the 480 Mbps line rate of the high-speed link the core
enumerates on, so the USB path cannot carry it. Measured on THIS Jetson: mode 50
delivers 32.8 fps (43 MB/s, ~the practical bulk ceiling) -- a little above the 30.17
the RPi managed, same bandwidth wall, slightly better host controller. A true 50 fps
has to come off the DVP path, not USB. Dropping resolution or changing pixel format
does not help: the core advertises only 1280x1024, and GREY at 1 byte/px is already
the cheapest of the four formats it offers (confirmed on this device: GREY, NV12,
YU12, YUYV, all 1280x1024 only).

The UVC descriptor is NOT a read-back. It reports a single discrete interval of
50.000 fps and keeps reporting it while the core is demonstrably delivering ~33, so
`v4l2-ctl --get-parm` cannot tell you the current rate. Only timing frames can.

Frame format is the 9-byte SD-IRM protocol:
    AA 05 00 <code> <lsb> <msb> <checksum> EB AA      checksum = sum(bytes 0..5) & 0xFF
and the core answers with the same code and a status byte, 0x00 = success.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

# Pin the port by the core's USB serial, not by ttyACMn. The device re-enumerates and
# the numbers drift. -if02 is the CDC-ACM control interface; -if04 is the UVC video node.
PORT_GLOB = "/dev/serial/by-id/usb-Artosyn_Sirius_*-if02"
# Extracts the serial out of a by-id port name so the video node can be matched to the
# SAME physical core -- with two cameras plugged in, "the only Sirius video node" is
# not a safe assumption.
PORT_SERIAL_RE = re.compile(r"usb-Artosyn_Sirius_(.+?)-if\d+$")

# Resolved in this order; first hit wins. See find_video().
THERMAL_LINK = "/dev/thermal0"                              # RPi's custom udev rule
V4L_BY_ID = "/dev/v4l/by-id/usb-Artosyn_Sirius_%s-video-index%d"   # stock udev
V4L_BY_ID_GLOB = "/dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index*"
CARD_MATCH = "sirius"                                       # /sys .../name, lowercased

BAUD = 115200
CODE = {25: 0x05, 50: 0x06}     # control code per rate; the core supports no others
SERVICE = "ir-tracker.service"


def find_port(explicit=None):
    if explicit:
        return explicit
    hits = sorted(glob.glob(PORT_GLOB))
    if not hits:
        sys.exit("no thermal core serial port found matching %s\n"
                 "is the camera plugged in? check: ls -l /dev/serial/by-id/" % PORT_GLOB)
    if len(hits) > 1:
        sys.exit("more than one core found:\n  %s\npick one with --port"
                 % "\n  ".join(hits))
    return hits[0]


def port_serial(port):
    """USB serial from a by-id port path, or None for a raw /dev/ttyACMn override."""
    if port is None:
        return None
    m = PORT_SERIAL_RE.search(os.path.realpath(port) if os.path.islink(port) else port)
    if m:
        return m.group(1)
    m = PORT_SERIAL_RE.search(port)
    return m.group(1) if m else None


def is_capture_node(dev):
    """True if dev can capture video. The core's second node is metadata-only: it
    enumerates no capture formats, so streaming from it fails after a long timeout
    instead of returning a rate. udev's ID_V4L_CAPABILITIES is the cheap check;
    if udevadm is unavailable, fall back to asking the driver for its formats."""
    try:
        r = subprocess.run(["udevadm", "info", "-q", "property", "-n", dev],
                           capture_output=True, text=True, timeout=10)
        for line in r.stdout.splitlines():
            if line.startswith("ID_V4L_CAPABILITIES="):
                return "capture" in line
    except Exception:                   # noqa: BLE001
        pass
    try:
        r = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats"],
                           capture_output=True, text=True, timeout=10)
        return "[0]:" in r.stdout       # at least one capture format enumerated
    except Exception:                   # noqa: BLE001
        return False


def find_video(explicit=None, serial=None, quiet=False):
    """Resolve the core's capture node. Returns a path, or None if nothing usable."""
    if explicit:
        if not os.path.exists(explicit):
            sys.exit("--dev %s does not exist" % explicit)
        return explicit

    # 1. The RPi's custom link, so this file stays a drop-in there.
    if os.path.exists(THERMAL_LINK):
        return THERMAL_LINK

    # 2. Stock udev's serial-pinned link, restricted to this core's serial when known.
    cands = []
    if serial:
        for idx in (0, 1):
            p = V4L_BY_ID % (serial, idx)
            if os.path.exists(p):
                cands.append(p)
    if not cands:
        cands = sorted(glob.glob(V4L_BY_ID_GLOB))

    # 3. Last resort: walk /dev/video* and match the driver-reported card name. Needed
    #    if /dev/v4l is missing entirely (udev's v4l rules not installed at all).
    if not cands:
        for node in sorted(glob.glob("/dev/video*")):
            name_file = "/sys/class/video4linux/%s/name" % os.path.basename(node)
            try:
                with open(name_file) as f:
                    if CARD_MATCH in f.read().strip().lower():
                        cands.append(node)
            except OSError:
                continue

    for c in cands:
        if is_capture_node(c):
            return c

    if not quiet:
        if cands:
            print("  found %d Sirius video node(s) but none can capture:\n    %s"
                  % (len(cands), "\n    ".join(cands)))
        else:
            print("  no Sirius video node found (looked at %s, then /dev/video*)"
                  % V4L_BY_ID_GLOB)
    return None


def build(code, lsb=0, msb=0):
    b = [0xAA, 0x05, 0x00, code, lsb, msb]
    b.append(sum(b) & 0xFF)
    return bytes(b + [0xEB, 0xAA])


def hx(b):
    return " ".join("%02X" % x for x in b)


def send(port, code, timeout=3.0):
    """Send one 9-byte frame, return (reply_bytes, error_string_or_None)."""
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed:  pip3 install pyserial")
    tx = build(code)
    try:
        with serial.Serial(port, BAUD, bytesize=8, parity="N", stopbits=1,
                           timeout=0.3) as ser:
            time.sleep(0.2)             # let the ACM link settle before the first write
            ser.reset_input_buffer()
            ser.write(tx)
            ser.flush()
            deadline = time.time() + timeout
            rx = b""
            while time.time() < deadline and len(rx) < 9:
                chunk = ser.read(9 - len(rx))
                if chunk:
                    rx += chunk
                    deadline = time.time() + timeout   # data flowing: keep waiting
    except Exception as e:                             # noqa: BLE001
        sys.exit("cannot open %s: %s\nin the dialout group? (id -nG)" % (port, e))

    print("  TX  %s" % hx(tx))
    print("  RX  %s" % (hx(rx) if rx else "(nothing)"))
    if not rx:
        return rx, "no reply -- core did not answer within %.0fs" % timeout
    if len(rx) < 9:
        return rx, "short reply: %d bytes, expected 9" % len(rx)
    if rx[0] != 0x55 or rx[1] != 0x05:
        return rx, "bad header: expected 55 05, got %02X %02X" % (rx[0], rx[1])
    if rx[7] != 0xEB or rx[8] != 0xAA:
        return rx, "bad tail: expected EB AA, got %02X %02X" % (rx[7], rx[8])
    cs = sum(rx[0:6]) & 0xFF
    if cs != rx[6]:
        return rx, "checksum mismatch: got %02X, computed %02X" % (rx[6], cs)
    if rx[3] != code:
        return rx, "wrong code echoed: sent %02X, got %02X" % (code, rx[3])
    if rx[2] != 0x00:
        # The protocol document lists no non-zero status codes, so there is nothing to
        # decode this into -- it means "failed, cause unspecified". Retrying is the
        # documented response.
        return rx, "status 0x%02X (non-zero = failure, cause unspecified)" % rx[2]
    return rx, None


def send_retry(rate, secs, explicit_port=None, gap=1.0):
    """Retry until the core answers or SECS elapse.

    The core does not answer on every enumeration: observed silent across ~40
    attempts on several instances, then answering twice in a row on another,
    with the video path healthy throughout. Since a replug sometimes lands in a
    talking state, retrying across replugs is more effective than any single
    hand-timed attempt. The port disappearing mid-run is expected here, not an
    error -- it just means the device is being replugged, so wait for it.
    """
    import serial as _serial          # noqa: F401  (fail early if missing)
    deadline = time.time() + secs
    code = CODE[rate]
    attempt = 0
    last = None
    print("retrying    up to %.0f s for %d fps (code 0x%02X); replug the camera"
          " to force a fresh enumeration" % (secs, rate, code))
    while time.time() < deadline:
        attempt += 1
        hits = sorted(glob.glob(PORT_GLOB)) if not explicit_port else [explicit_port]
        if not hits:
            if last != "absent":
                print("  waiting for the port to appear ...")
                last = "absent"
            time.sleep(gap)
            continue
        port = hits[0]
        try:
            rx, err = send_quiet(port, code)
        except Exception as e:                       # noqa: BLE001
            rx, err = b"", "%s" % type(e).__name__
        if err is None:
            print("  attempt %d on %s" % (attempt, port))
            print("  RX  %s" % hx(rx))
            return rx, None, port
        if err != last:
            print("  attempt %d: %s" % (attempt, err))
            last = err
        time.sleep(gap)
    return b"", ("no reply after %.0f s (%d attempts) -- the core never answered"
                 % (secs, attempt)), None


def send_quiet(port, code, timeout=2.0):
    """send() without the per-attempt TX/RX printing, for the retry loop."""
    import serial
    tx = build(code)
    with serial.Serial(port, BAUD, bytesize=8, parity="N", stopbits=1,
                       timeout=0.3) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(tx)
        ser.flush()
        deadline = time.time() + timeout
        rx = b""
        while time.time() < deadline and len(rx) < 9:
            chunk = ser.read(9 - len(rx))
            if chunk:
                rx += chunk
                deadline = time.time() + timeout
    if not rx:
        return rx, "no reply"
    if len(rx) < 9:
        return rx, "short reply (%d bytes)" % len(rx)
    if rx[0] != 0x55 or rx[1] != 0x05:
        return rx, "bad header %02X %02X" % (rx[0], rx[1])
    if (sum(rx[0:6]) & 0xFF) != rx[6]:
        return rx, "checksum mismatch"
    if rx[3] != code:
        return rx, "wrong code echoed %02X" % rx[3]
    if rx[2] != 0x00:
        return rx, "status 0x%02X (failure)" % rx[2]
    return rx, None


def measure(frames=200, dev=None):
    """Time frames actually delivered. Needs the camera free -- see service_active()."""
    if dev is None:
        return None                     # find_video() already explained why
    cmd = ["v4l2-ctl", "-d", dev,
           "--set-fmt-video=width=1280,height=1024,pixelformat=GREY",
           "--stream-mmap", "--stream-count=%d" % frames, "--stream-to=/dev/null"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        # v4l2-ctl writes the running "<<<<< NN.NN fps" progress to STDERR, not stdout.
        # Read both, or the rate is silently never found and this looks like a device
        # conflict instead of a parsing bug.
        out = r.stdout + r.stderr
    except FileNotFoundError:
        print("  cannot measure: v4l2-ctl not installed (apt install v4l-utils)")
        return None
    except subprocess.TimeoutExpired:
        print("  cannot measure: capture timed out -- is another process streaming?")
        return None
    # v4l2-ctl prints a running "<<<<< NN.NN fps" line per ~batch; the last is settled.
    rates = [float(m) for m in re.findall(r"([\d.]+) fps", out)]
    if not rates:
        print("  cannot measure: no frames captured. Another process may hold %s" % dev)
        return None
    print("  measured    %.2f fps  (%d frames, %s -> %s)"
          % (rates[-1], frames, dev, os.path.realpath(dev)))
    return rates[-1]


def service_installed():
    """Whether the unit exists at all. On a host without it the running-service guard
    below can never fire, and a safety check that is silently vacuous is worth saying
    out loud rather than letting it read as 'checked, all clear'."""
    try:
        r = subprocess.run(["systemctl", "list-unit-files", SERVICE],
                           capture_output=True, text=True, timeout=10)
        return SERVICE in r.stdout
    except Exception:                   # noqa: BLE001
        return False


def service_active():
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", SERVICE],
                              timeout=10).returncode == 0
    except Exception:                   # noqa: BLE001
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rate", nargs="?", type=int, choices=sorted(CODE),
                   help="target frame rate: 25 or 50 (the core supports no others)")
    ap.add_argument("--show", action="store_true",
                   help="change nothing; just time the rate being delivered now")
    ap.add_argument("--measure", action="store_true",
                   help="after setting, time the frames actually delivered")
    ap.add_argument("--list", action="store_true",
                   help="show which device nodes this core resolves to, then exit")
    ap.add_argument("--frames", type=int, default=200,
                   help="frames to time when measuring (default 200)")
    ap.add_argument("--port", help="override the auto-detected serial port")
    ap.add_argument("--dev", help="override the auto-detected video capture node")
    ap.add_argument("--force", action="store_true",
                   help="proceed even though %s is running" % SERVICE)
    ap.add_argument("--retry", type=float, metavar="SECS", default=0,
                   help="keep retrying for SECS until the core answers, waiting for\n"
                        "the port to appear if it is absent. The core has been seen\n"
                        "to answer on only some enumerations, so this catches the\n"
                        "window after a replug instead of needing a hand-timed run.")
    a = ap.parse_args()

    if a.list:
        port = a.port or (sorted(glob.glob(PORT_GLOB)) or [None])[0]
        print("serial port %s" % (port or "(none found)"))
        if port and os.path.islink(port):
            print("            -> %s" % os.path.realpath(port))
        print("usb serial  %s" % (port_serial(port) or "(unknown)"))
        dev = find_video(a.dev, port_serial(port))
        print("video node  %s" % (dev or "(none found)"))
        if dev and os.path.islink(dev):
            print("            -> %s" % os.path.realpath(dev))
        print("%-11s %s" % (SERVICE,
              "active" if service_active() else
              ("inactive" if service_installed() else "not installed on this host")))
        return 0 if (port and dev) else 1

    if a.show:
        if a.rate is not None:
            ap.error("--show changes nothing; drop the rate argument")
        # --show never opens the serial port, but it still reads it to learn the serial
        # so the video node is matched to the same core. Missing port is not fatal here.
        port = a.port or (sorted(glob.glob(PORT_GLOB)) or [None])[0]
        print("current delivered rate:")
        return 0 if measure(a.frames, find_video(a.dev, port_serial(port))) else 1
    if a.rate is None:
        ap.error("give a rate (25 or 50), --show, or --list")

    # Changing the rate under the running pipeline changes frame timing beneath it.
    # --latency-ms 200 (the ego-motion camera delay) was measured at ~30 fps, and the
    # tracker's gate and acquisition tests assume that inter-frame interval, so a
    # silent switch to 25 fps mid-sortie shifts the ego-motion compensation.
    if service_active() and not a.force:
        sys.exit("%s is running -- it is streaming from the camera.\n"
                 "Changing the frame rate under it shifts the inter-frame interval\n"
                 "(40 ms at 25 fps vs 33 ms at 30), which its --latency-ms 200\n"
                 "ego-motion delay was tuned against.\n\n"
                 "  sudo systemctl stop %s\n"
                 "  %s %d --force\n"
                 % (SERVICE, SERVICE, sys.argv[0], a.rate))
    if not service_installed():
        print("note        %s not installed here, so the running-pipeline guard\n"
              "            below is a no-op -- nothing checked whether something else\n"
              "            is streaming from the core." % SERVICE)

    if a.retry > 0:
        rx, err, port = send_retry(a.rate, a.retry, a.port)
        if port is None:
            port = a.port          # nothing answered; fall back to the override
    else:
        port = find_port(a.port)
        print("port        %s" % port)
        print("setting     %d fps (control code 0x%02X)" % (a.rate, CODE[a.rate]))
        rx, err = send(port, CODE[a.rate])
    if err:
        print("FAILED: %s" % err)
        return 1
    print("  ok          status 0x00, code 0x%02X echoed, checksum valid" % rx[3])

    if a.measure:
        if service_active():
            print("  skipping measurement: %s holds the camera" % SERVICE)
        else:
            time.sleep(0.5)          # let the core settle before timing it
            got = measure(a.frames, find_video(a.dev, port_serial(port)))
            if got is not None and a.rate == 50 and got < 40:
                print("  NOTE: ~33 fps is expected in mode 50 -- USB cannot carry 50.")
    elif a.rate == 50:
        print("  NOTE: mode 50 delivers ~33 fps over USB, not 50 (bandwidth-limited).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
