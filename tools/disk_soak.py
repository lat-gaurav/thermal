#!/usr/bin/env python3
"""Can this disk take 25 fps of raw frames without dropping any?

    python3 ssd_frame_test.py                      60 s onto /mnt/external_ssd
    python3 ssd_frame_test.py --seconds 300        the sortie-length run
    python3 ssd_frame_test.py --dir /home/rpi5/flight-logs   the SD-card fallback

It drives the REAL flight/rawrec.py writer, from a producer thread pacing at
config.CAMERA_FPS exactly as flight/camera.py's grab thread does, so a pass here
means the recording path passes -- not that the disk benchmarks well. dd does not
answer this question: dd writes into page cache and reports the cache's speed,
which is how a device that cannot hold a USB link still measured 431 MB/s.

WHAT COUNTS AS A DROP. RawRecWriter.offer() copies into a bounded queue of
RAW_QDEPTH_FRAMES and returns; a writer thread drains it. Queue full means the
frame is discarded and counted. At qdepth 8 and 25 fps that is 320 ms of slack,
so ANY write stall past ~0.32 s costs frames. The pass mark is therefore zero,
not "a few".

IT ALSO WATCHES THE BUS. A USB mass-storage device that re-enumerates mid-run
takes the filesystem with it, and the writer sees that as a write error rather
than as backpressure -- so the drop counter alone would report a clean run right
up until the file vanished. The devnum of the backing device is sampled every
second and any change is a hard fail.
"""
import argparse
import os
import pathlib
import subprocess
import sys
import threading
import time

import numpy as np

_REPO = pathlib.Path("/home/rpi5/thermal")
sys.path.insert(0, str(_REPO))
import config                                    # noqa: E402
from flight.rawrec import RawRecWriter           # noqa: E402


# --------------------------------------------------------------------------- #
# power and bus state: the preflight that would have saved the last run
# --------------------------------------------------------------------------- #
def _dt(name):
    """One value out of /sys/firmware/devicetree/base/chosen/power/."""
    p = pathlib.Path("/sys/firmware/devicetree/base/chosen/power") / name
    try:
        return p.read_bytes()
    except OSError:
        return None


def _be32(b):
    return None if not b else int.from_bytes(b[:4], "big")


def power_report():
    """-> (lines, ok). ok is False when the USB budget is the 600 mA default."""
    lines, ok = [], True

    hi = _be32(_dt("usb_max_current_enable"))
    pd = _dt("usbpd_power_data_objects")
    mx = _be32(_dt("max_current"))
    oc = _be32(_dt("usb_over_current_detected"))

    # THE FLAG IS THE AUTHORITATIVE ANSWER, not the PD contract. This rig is
    # powered from the GPIO 5V pins, so there is no USB-C connector and PD can
    # never negotiate -- usbpd_power_data_objects reads all zeros by
    # construction, forever. Treating that as a failure (an earlier version of
    # this file did) refuses to run on a board that is in fact configured
    # correctly. PD only matters as the OTHER way the budget gets unlocked.
    pd_ok = bool(pd and any(pd))
    if hi:
        lines.append("  usb high-current   ENABLED -- ports share 1.6 A")
        lines.append("                     (config.txt usb_max_current_enable=1)")
    elif pd_ok:
        lines.append("  usb high-current   unlocked by PD negotiation")
    else:
        lines.append("  usb high-current   OFF -- ports share 600 mA TOTAL")
        lines.append("                     a portable SSD alone draws ~0.9-1.5 A writing")
        lines.append("                     set usb_max_current_enable=1 in config.txt")
        ok = False

    lines.append("  usb-pd             %s" % (
        "negotiated (%d objects)" % (len(pd) // 4) if pd_ok
        else "absent -- expected when powered from the GPIO 5V pins"))
    lines.append("  firmware max_current  %s mA" % ("?" if mx is None else mx))
    if oc:
        lines.append("  OVER-CURRENT LATCHED -- a port has already tripped")
        ok = False

    try:
        t = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                           text=True, timeout=5).stdout.strip()
        lines.append("  %s%s" % (t, "" if t.endswith("0x0") else "   <-- NOT CLEAN"))
    except Exception:
        pass
    try:
        v = subprocess.run(["vcgencmd", "pmic_read_adc", "EXT5V_V"],
                           capture_output=True, text=True, timeout=5).stdout.split("=")[-1]
        volts = float(v.strip().rstrip("V"))
        # 5V +/-5% is 4.75-5.25. Below 4.75 the board is out of spec and a USB
        # brownout is a question of when; between 4.75 and 4.85 there is little
        # headroom left for the droop that arrives with the write load, which on
        # a GPIO-fed board is mostly wiring resistance rather than the supply.
        note = ""
        if volts < 4.75:
            note = "   <-- OUT OF SPEC (4.75 V floor)"
            ok = False
        elif volts < 4.85:
            note = "   <-- little headroom under load"
        lines.append("  EXT5V              %.3f V%s" % (volts, note))
    except Exception:
        pass
    return lines, ok


def backing_device(path):
    """The /sys/bus/usb/devices/<x> entry behind a path, or None if not USB.

    Re-enumeration is what has to be caught, and the kernel name (sda) is reused
    across it -- the devnum is not, so that is what gets watched.
    """
    try:
        src = subprocess.run(["findmnt", "-n", "-o", "SOURCE", "--target", str(path)],
                             capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return None, None
    if not src.startswith("/dev/"):
        return None, src
    base = os.path.basename(src).rstrip("0123456789")
    for usb in pathlib.Path("/sys/bus/usb/devices").glob("*-*"):
        if (usb / "devnum").exists() and list(usb.rglob("block/%s" % base)):
            return usb, src
    return None, src


def devnum_of(usbdir):
    try:
        return int((usbdir / "devnum").read_text().strip())
    except OSError:
        return None


# --------------------------------------------------------------------------- #
# the run
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default="/mnt/external_ssd/logs",
                    help="where to write the test file (default %(default)s)")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--fps", type=float, default=float(config.CAMERA_FPS))
    ap.add_argument("--width", type=int, default=config.CAMERA_WIDTH)
    ap.add_argument("--height", type=int, default=config.CAMERA_HEIGHT)
    ap.add_argument("--keep", action="store_true", help="do not delete the file after")
    ap.add_argument("--force", action="store_true",
                    help="run even if the power preflight fails")
    ap.add_argument("--camera", nargs="?", const=config.CAMERA_DEVICE, default=None,
                    metavar="DEV",
                    help="record the REAL camera instead of synthetic frames "
                         "(default device %s). This is the flight condition: the "
                         "core and the disk draw from the same 5V budget at the "
                         "same time, and the recorder is fed from the grab thread "
                         "exactly as flight_pipeline.py feeds it."
                         % config.CAMERA_DEVICE)
    args = ap.parse_args()

    W, H, FPS = args.width, args.height, args.fps
    fb = W * H
    need_mbs = fb * FPS / 1e6
    out_dir = pathlib.Path(args.dir)

    print("=" * 72)
    print("raw recording soak test")
    print("  frames      %dx%d GREY at %.0f fps = %.2f MB/s sustained"
          % (W, H, FPS, need_mbs))
    print("  duration    %.0f s  -> %.1f GB" % (args.seconds, need_mbs * args.seconds / 1000))
    print("  queue       %d frames = %.0f ms of slack before a drop"
          % (config.RAW_QDEPTH_FRAMES, 1000 * config.RAW_QDEPTH_FRAMES / FPS))

    print("-" * 72)
    print("power")
    plines, pok = power_report()
    print("\n".join(plines))
    if not pok and not args.force:
        print("-" * 72)
        print("REFUSING TO RUN: the USB power budget is the limiting factor, not the")
        print("disk. A run now measures how fast the device falls off the bus.")
        print("Fix the supply first, or pass --force to measure it anyway.")
        return 2

    print("-" * 72)
    print("target")
    if not out_dir.is_dir():
        print("  %s does not exist" % out_dir)
        return 2
    st = os.statvfs(out_dir)
    free_gb = st.f_bavail * st.f_frsize / 1e9
    root_st = os.statvfs("/")
    on_root = (st.f_fsid == root_st.f_fsid) or (
        os.stat(out_dir).st_dev == os.stat("/").st_dev)
    usbdir, src = backing_device(out_dir)
    print("  path        %s" % out_dir)
    print("  device      %s%s" % (src or "?",
                                  "  (USB %s)" % usbdir.name if usbdir else ""))
    print("  free        %.1f GB  = %.0f min of raw" % (free_gb, free_gb * 1000 / (need_mbs * 60)))
    if on_root:
        print("  WARNING     this is the ROOT filesystem -- filling it takes the")
        print("              journal and the pipeline down, not just the recording")
    if free_gb * 1000 < need_mbs * args.seconds * 1.2:
        print("  not enough free space for this run")
        return 2

    dev0 = devnum_of(usbdir) if usbdir else None
    if dev0 is not None:
        print("  usb devnum  %d  (any change during the run is a hard fail)" % dev0)

    cam, frame = None, None
    if args.camera:
        from flight.camera import FlightCamera
        print("-" * 72)
        print("camera")
        try:
            cam = FlightCamera(args.camera, W, H).start()
        except RuntimeError as e:
            print("  %s" % e)
            return 2
        print("  device      %s  %dx%d GREY" % (args.camera, W, H))
        print("  note        the core and the disk now share the 5V budget --")
        print("              this is the condition the rig actually flies in")
    else:
        # A realistic frame: random bytes, so nothing downstream can get clever
        # about runs of zeros and the memcpy cost is honest. Generated once --
        # the test is of the disk, not of numpy.
        rng = np.random.default_rng(0xC0FFEE)
        frame = rng.integers(0, 256, size=(H, W), dtype=np.uint8)

    path = out_dir / ("ssd-soak-%s.rawrec" % time.strftime("%Y%m%d-%H%M%S"))
    rec = RawRecWriter(path, W, H, bpp=1, pixfmt="GREY",
                       reserve_mb=config.RAW_RESERVE_MB,
                       meta={"source": args.camera or "disk_soak",
                             "synthetic": cam is None}).open()

    print("-" * 72)
    print("running -- ^C to stop early", flush=True)

    bus_fail = []
    volts = []
    stop = threading.Event()

    def watch_bus():
        """Sample the devnum and the 5V rail every second.

        The devnum is the check dd never did: a device that re-enumerates takes
        the filesystem with it, and the drop counter alone would call that a
        clean run right up until the file vanished.

        The rail is sampled because on a GPIO-fed board the write load is when
        the droop arrives, and the idle reading says nothing about it.
        """
        while not stop.wait(1.0):
            try:
                v = subprocess.run(["vcgencmd", "pmic_read_adc", "EXT5V_V"],
                                   capture_output=True, text=True,
                                   timeout=5).stdout.split("=")[-1]
                volts.append(float(v.strip().rstrip("V")))
            except Exception:
                pass
            if usbdir is None:
                continue
            d = devnum_of(usbdir)
            if d is None:
                bus_fail.append("device vanished from the bus")
                return
            if dev0 is not None and d != dev0:
                bus_fail.append("re-enumerated: devnum %d -> %d" % (dev0, d))
                return

    watcher = threading.Thread(target=watch_bus, daemon=True)
    watcher.start()

    offer_ms = []
    t0 = time.monotonic()
    last_print = t0

    def tick(now):
        nonlocal last_print
        if now - last_print < 5.0:
            return
        last_print = now
        extra = "  grabbed %5d" % cam.grabbed if cam else ""
        print("  t+%5.1fs  offered %5d  written %5d  DROPPED %d  %.1f MB/s%s"
              % (now - t0, rec.offered, rec.frames, rec.dropped,
                 rec.bytes_written / 1e6 / (now - t0), extra), flush=True)

    try:
        if cam is not None:
            # THE RECORDER IS FED FROM THE GRAB THREAD, which is how
            # flight_pipeline.py feeds it -- so this measures the real path,
            # including the core competing for the same 5V budget. Nothing
            # paces it here: the camera's own rate is the rate under test.
            cam.sink = rec
            while time.monotonic() - t0 < args.seconds and not bus_fail:
                if not rec.active:
                    break
                time.sleep(0.2)
                tick(time.monotonic())
            cam.sink = None
        else:
            # The producer paces off an ABSOLUTE schedule rather than sleeping a
            # fixed interval, so a slow offer() cannot make the run drift long and
            # quietly lower the rate it claims to be testing.
            period = 1.0 / FPS
            n = 0
            while True:
                if time.monotonic() - t0 >= args.seconds or bus_fail:
                    break
                a = time.monotonic()
                rec.offer(frame, t_mono=a)
                offer_ms.append(1000.0 * (time.monotonic() - a))
                n += 1
                if not rec.active:
                    break
                tick(time.monotonic())
                slack = (t0 + n * period) - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
    except KeyboardInterrupt:
        print("\n  interrupted", flush=True)
        if cam is not None:
            cam.sink = None

    stop.set()
    elapsed = time.monotonic() - t0
    stats = rec.close()
    if cam is not None:
        cam.stop()

    offer_ms.sort()
    def pct(p):
        return offer_ms[min(len(offer_ms) - 1, int(p * len(offer_ms)))] if offer_ms else 0.0

    print("-" * 72)
    print("result")
    print("  elapsed     %.1f s" % elapsed)
    print("  offered     %d frames (%.2f fps)" % (stats["offered"], stats["offered"] / elapsed))
    print("  written     %d frames, %.2f GB (%.2f MB/s)"
          % (stats["frames"], stats["gb"], stats["gb"] * 1000 / elapsed))
    print("  DROPPED     %d" % stats["dropped"])
    if cam is not None:
        print("  camera      %d grabbed, %d read failures" % (cam.grabbed, cam.read_fail))
    if offer_ms:
        print("  offer()     p50 %.2f ms   p99 %.2f ms   max %.2f ms"
              % (pct(.50), pct(.99), offer_ms[-1]))
    if volts:
        # The minimum is the number that matters: one dip under 4.75 V is all it
        # takes to reset a port, and an average would hide it completely.
        lo = min(volts)
        print("  EXT5V       min %.3f V   mean %.3f V   (%d samples)%s"
              % (lo, sum(volts) / len(volts), len(volts),
                 "   <-- WENT OUT OF SPEC" if lo < 4.75
                 else "   <-- no margin left" if lo < 4.78 else ""))
    if stats["stop_reason"]:
        print("  writer      STOPPED: %s" % stats["stop_reason"])
    if bus_fail:
        print("  BUS         %s" % bus_fail[0])

    if not args.keep:
        try:
            os.unlink(path)
            print("  file        removed (pass --keep to retain)")
        except OSError as e:
            print("  file        could NOT be removed: %s" % e)
    else:
        print("  file        %s" % path)

    print("=" * 72)
    ok = (stats["dropped"] == 0 and not bus_fail and not stats["stop_reason"]
          and stats["frames"] >= 0.99 * stats["offered"])
    print("PASS -- the disk holds %.0f fps with no dropped frames" % FPS if ok
          else "FAIL -- see above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
