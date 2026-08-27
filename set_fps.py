#!/usr/bin/env python3
"""Set the Artosyn "Sirius" thermal core's frame rate over its USB serial port.

    set_fps.py 25                 switch the core to 25 fps
    set_fps.py 50                 switch it back to 50 fps (the power-on default)
    set_fps.py 25 --measure       set it, then time the frames actually delivered
    set_fps.py --show             don't change anything; just time the current rate

The core accepts only these two rates: control code 0x05 = 25 fps, 0x06 = 50 fps.
There is NO read-back command for frame rate, so --show/--measure time delivered
frames instead; that is the only way to know what the core is doing.

WHAT 50 fps ACTUALLY GIVES YOU OVER USB: about 30, not 50. 1280x1024 GREY at 50 fps
is 65.5 MB/s = 524 Mbps, above the 480 Mbps line rate of the high-speed link the core
enumerates on, so the USB path cannot carry it. Measured on this hardware: mode 50
delivers 30.17 fps (39.5 MB/s, ~the practical bulk ceiling), mode 25 delivers 25.04 --
i.e. 25 is the one rate that is actually honoured end to end. A true 50 fps has to come
off the DVP path, not USB. Dropping resolution or changing pixel format does not help:
the core advertises only 1280x1024, and GREY at 1 byte/px is already the cheapest of
the four formats it offers.

The UVC descriptor is NOT a read-back. It reports a single discrete interval of
50.000 fps and keeps reporting it while the core is demonstrably delivering 25, so
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
# the numbers drift -- the same reason 99-thermal-cam.rules pins the video node by
# ATTRS{serial}. -if02 is the CDC-ACM control interface; -if04 is the UVC video node.
PORT_GLOB = "/dev/serial/by-id/usb-Artosyn_Sirius_*-if02"
VIDEO_DEV = "/dev/thermal0"

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


def measure(frames=200, dev=VIDEO_DEV):
    """Time frames actually delivered. Needs the camera free -- see service_active()."""
    if not os.path.exists(dev):
        print("  cannot measure: %s does not exist" % dev)
        return None
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
    print("  measured    %.2f fps  (%d frames, %s)" % (rates[-1], frames, dev))
    return rates[-1]


def service_active():
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", SERVICE],
                              timeout=10).returncode == 0
    except Exception:                    # noqa: BLE001
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
    ap.add_argument("--frames", type=int, default=200,
                   help="frames to time when measuring (default 200)")
    ap.add_argument("--port", help="override the auto-detected serial port")
    ap.add_argument("--force", action="store_true",
                   help="proceed even though %s is running" % SERVICE)
    a = ap.parse_args()

    if a.show:
        if a.rate is not None:
            ap.error("--show changes nothing; drop the rate argument")
        print("current delivered rate:")
        return 0 if measure(a.frames) else 1
    if a.rate is None:
        ap.error("give a rate (25 or 50), or --show")

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
            got = measure(a.frames)
            if got is not None and a.rate == 50 and got < 40:
                print("  NOTE: ~30 fps is expected in mode 50 -- USB cannot carry 50.")
    elif a.rate == 50:
        print("  NOTE: mode 50 delivers ~30 fps over USB, not 50 (bandwidth-limited).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
