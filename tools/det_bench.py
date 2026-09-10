#!/usr/bin/env python3
"""det_bench.py -- measure detector cost per frame on a .rawrec recording.

    tools/det_bench.py logs/flight-20260908-165326-db6f3cbe-01.rawrec
    tools/det_bench.py FILE.rawrec -n 500 --from 1800     500 frames from 1800
    tools/det_bench.py FILE.rawrec --threads 1            pin OpenCV to 1 core
    tools/det_bench.py FILE.rawrec --tophat               also bench tophat_scr

WHY THIS EXISTS. thermal_detect's docstring claims "~16 ms for 1280x1024".
That is a single number with no provenance -- no percentiles, no separation of
disk from compute, and no note of how many cores it assumed. On a 25 Hz capture
the budget is 40 ms/frame, so the question is not the mean, it is the tail: a
p99 over budget drops frames even when the mean looks comfortable.

WHAT IS TIMED SEPARATELY, and why each is its own number:

  read      cap.frame(i) -- seek + 1.31 MB read off the SSD. Kept apart from
            compute because it is I/O bound and page-cache dependent, so mixing
            it into the detector's number makes the detector look slower on a
            cold cache and faster on a warm one.
  features  the four whole-frame filters. This is where the time goes.
  tail      detect()'s ranking and greedy merge, the only part whose cost
            depends on the scene -- an empty frame exits early, a speckle field
            walks hundreds of candidates.
  detect    features + tail, the number that actually has to fit in 40 ms.

COMPUTE IS TIMED ON RAM-RESIDENT FRAMES. Frames are read once into a list
before timing starts, so the per-stage numbers are pure compute with no I/O
underneath. The read cost is measured on its own pass instead.

THERMAL THROTTLING IS CHECKED, NOT ASSUMED. This Pi 5 starts its fan at 50 C
and soft-throttles the ARM core near 80 C. A few hundred 1280x1024 median
blurs will heat it, so temperature and the firmware's throttle word are sampled
before and after; if the core throttled mid-run the timings are not comparable
and the report says so rather than quietly reporting a slow tail.
"""
import argparse
import os
import statistics
import subprocess
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as fh:
            return int(fh.read()) / 1000.0
    except OSError:
        return float("nan")


def throttled():
    """The firmware's throttle word. 0x0 means nothing has been limited."""
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=5).stdout.strip()
        return out.split("=")[-1] if "=" in out else "?"
    except (OSError, subprocess.SubprocessError):
        return "?"


def arm_mhz():
    try:
        out = subprocess.run(["vcgencmd", "measure_clock", "arm"],
                             capture_output=True, text=True, timeout=5).stdout
        return int(out.strip().split("=")[-1]) / 1e6
    except (OSError, subprocess.SubprocessError, ValueError):
        return float("nan")


def stats(name, xs, budget=None):
    """One line of timing. Percentiles, because the tail is the whole point."""
    xs = sorted(xs)
    n = len(xs)
    p = lambda q: xs[min(n - 1, int(q * n))]
    line = ("  %-10s mean %7.2f  med %7.2f  p95 %7.2f  p99 %7.2f  max %7.2f ms"
            % (name, 1000 * statistics.fmean(xs), 1000 * p(0.50),
               1000 * p(0.95), 1000 * p(0.99), 1000 * xs[-1]))
    if budget:
        over = sum(1 for x in xs if x > budget)
        line += "   over budget: %d/%d (%.1f%%)" % (over, n, 100.0 * over / n)
    print(line)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("-n", "--frames", type=int, default=200,
                    help="frames to time (default 200)")
    ap.add_argument("--from", dest="first", type=int, default=1, metavar="N",
                    help="first frame, 1-based (default 1)")
    ap.add_argument("--every", type=int, default=1, metavar="N",
                    help="take every Nth frame (default 1)")
    ap.add_argument("--threads", type=int, metavar="N",
                    help="pin OpenCV to N threads (default: leave at %d)"
                         % cv2.getNumThreads())
    ap.add_argument("--warmup", type=int, default=5, metavar="N",
                    help="untimed frames first, to fault in code paths "
                         "and let the CPU clock up (default 5)")
    ap.add_argument("--tophat", action="store_true",
                    help="also bench detector/tophat_scr for comparison")
    ap.add_argument("--fps", type=float, default=25.0, metavar="F",
                    help="frame rate the budget is derived from (default 25)")
    args = ap.parse_args()

    if args.threads:
        cv2.setNumThreads(args.threads)
    budget = 1.0 / args.fps

    import thermal_detect as td
    from rawrec_view import Capture

    cap = Capture(args.file, quiet=True)
    first = max(0, args.first - 1)
    idx = list(range(first, min(len(cap), first + args.frames * args.every),
                     args.every))
    if not idx:
        raise SystemExit("no frames in that range")

    print("recording  %s" % os.path.basename(args.file))
    print("           %dx%d %s, %d frames, measured %.3f fps"
          % (cap.w, cap.h, cap.dtype, len(cap), cap.fps))
    print("timing     %d frames from %d, every %d" % (len(idx), first + 1, args.every))
    print("opencv     %s, %d thread(s) of %d cpus"
          % (cv2.__version__, cv2.getNumThreads(), cv2.getNumberOfCPUs()))
    print("budget     %.1f ms/frame at %.1f fps" % (1000 * budget, args.fps))
    t0, thr0, mhz0 = cpu_temp(), throttled(), arm_mhz()
    print("start      %.1f C, arm %.0f MHz, throttled=%s" % (t0, mhz0, thr0))

    # ---- read pass: I/O only, and it is its own number for a reason ----
    print("\nreading %d frames (%.1f MB) ..."
          % (len(idx), len(idx) * cap.nbytes / 1e6), end="", flush=True)
    reads, frames = [], []
    for i in idx:
        t = time.perf_counter()
        f = cap.frame(i)
        reads.append(time.perf_counter() - t)
        frames.append(f)
    print(" done")

    # ---- warmup: first call pays for kernel allocation and clock ramp ----
    for f in frames[:args.warmup]:
        td.detect(f)

    # ---- per-stage compute, on RAM-resident frames ----
    RING, BOX = td.RING, td.BOX
    t_feat, t_det, t_tail = [], [], []
    t_med, t_dil_ring, t_dil_box, t_box = [], [], [], []
    ndets = []
    for f in frames:
        t = time.perf_counter(); td.features(f); t_feat.append(time.perf_counter() - t)

        t = time.perf_counter(); cv2.medianBlur(f, td.MED_W); t_med.append(time.perf_counter() - t)
        t = time.perf_counter(); cv2.dilate(f, RING);          t_dil_ring.append(time.perf_counter() - t)
        t = time.perf_counter(); cv2.dilate(f, BOX);           t_dil_box.append(time.perf_counter() - t)
        cand = (f == cv2.dilate(f, BOX))
        t = time.perf_counter()
        cv2.boxFilter(cand.astype(np.float32), -1, (td.CROWD_W, td.CROWD_W),
                      normalize=False)
        t_box.append(time.perf_counter() - t)

        t = time.perf_counter(); d = td.detect(f); dt = time.perf_counter() - t
        t_det.append(dt); ndets.append(len(d))
        t_tail.append(max(0.0, dt - t_feat[-1]))

    t1, thr1, mhz1 = cpu_temp(), throttled(), arm_mhz()

    print("\nthermal_detect.detect  (%d frames, %.2f detections/frame)"
          % (len(frames), statistics.fmean(ndets)))
    stats("read", reads)
    stats("features", t_feat)
    stats("tail", t_tail)
    stats("detect", t_det, budget)
    print("\n  stage breakdown inside features()")
    stats("median31", t_med)
    stats("dilate-ring", t_dil_ring)
    stats("dilate-nms", t_dil_box)
    stats("box101", t_box)

    print("\n  read + detect  mean %.2f ms  ->  %.1f fps sustained"
          % (1000 * (statistics.fmean(reads) + statistics.fmean(t_det)),
             1.0 / (statistics.fmean(reads) + statistics.fmean(t_det))))

    if args.tophat:
        from detector.tophat_scr import detect as th_detect
        th_detect(frames[0])
        t_th = []
        for f in frames:
            t = time.perf_counter(); th_detect(f); t_th.append(time.perf_counter() - t)
        print("\ndetector/tophat_scr.detect  (SE_SIZE=%d, OUTER=%d)"
              % (__import__("detector.tophat_scr", fromlist=["SE_SIZE"]).SE_SIZE,
                 __import__("detector.tophat_scr", fromlist=["OUTER_SIZE"]).OUTER_SIZE))
        stats("detect", t_th, budget)

    print("\nend        %.1f C (+%.1f), arm %.0f MHz, throttled=%s"
          % (t1, t1 - t0, mhz1, thr1))
    if thr1 not in ("0x0", "?") and thr1 != thr0:
        print("  WARNING: the core throttled during this run (%s -> %s)."
              "  Timings above are not comparable -- let it cool and re-run."
              % (thr0, thr1))
    cap.close()


if __name__ == "__main__":
    main()
