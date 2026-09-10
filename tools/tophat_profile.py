#!/usr/bin/env python3
"""Profile detector/tophat_scr.py stage by stage, and price the ways to speed it up.

    tools/tophat_profile.py                       grab frames off the live camera
    tools/tophat_profile.py --npy frames.npy      or profile a saved stack
    tools/tophat_profile.py --roi 520x300         profile the size the tracker uses
    tools/tophat_profile.py --se-scan             cost vs DETECTOR_SE_SIZE
    tools/tophat_profile.py --variants            price alternatives, and check each
                                                  one against the real detector

WHY THIS EXISTS SEPARATELY FROM det_bench.py. det_bench profiles
thermal_detect (the root-level detector) against a .rawrec and times tophat_scr
only as one opaque number. This one breaks tophat_scr open, because for that
detector the interesting fact is that ONE of its calls is ~92% of the runtime,
and the second interesting fact is that the cost depends on a config value
(DETECTOR_SE_SIZE) in a way nothing else in the repo surfaces: quadratically.

WHAT --variants IS FOR. Each alternative is timed AND compared against the real
detector on the same frames, because a speedup that changes which boxes come out
is not a speedup, it is a different detector. The comparison reports box
geometry separately from score, since a score that moves in its last digits is
float reassociation and a box that moves is a behaviour change.
"""
import argparse
import os
import statistics
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from detector.tophat_scr import detect, SE_SIZE, OUTER_SIZE, MIN_SCR, NOISE_FLOOR
from detector.tophat_scr import MIN_AREA, MAX_AREA, INNER_AREA, OUTER_AREA


def grab(device, n, width, height):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit("%s: could not open (held by another process? stop it first)" % device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"GREY"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    out, t0 = [], time.time()
    while len(out) < n and time.time() - t0 < 20:
        ok, f = cap.read()
        if ok:
            out.append(f.copy())
    cap.release()
    if not out:
        sys.exit("%s: opened but delivered no frames" % device)
    return np.stack(out)


def bench(fn, arg, reps):
    fn(arg)
    ts = []
    for _ in range(reps):
        t = time.perf_counter()
        fn(arg)
        ts.append(time.perf_counter() - t)
    return statistics.fmean(ts), sorted(ts)


def line(label, mean, total=None):
    s = "  %-38s %8.1f ms" % (label, 1000 * mean)
    if total:
        s += "   %5.1f%%" % (100 * mean / total)
    print(s)


def stages(frame, reps):
    """Time each step of detect() on this frame, in the order detect() runs them."""
    se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (SE_SIZE, SE_SIZE))
    tophat = cv2.morphologyEx(frame, cv2.MORPH_TOPHAT, se).astype(np.float32)
    inner = cv2.blur(tophat, (SE_SIZE, SE_SIZE))
    outer = cv2.blur(tophat, (OUTER_SIZE, OUTER_SIZE))
    w_out = OUTER_AREA / (OUTER_AREA - INNER_AREA)
    w_in = -INNER_AREA / (OUTER_AREA - INNER_AREA)
    denom = cv2.addWeighted(outer, w_out, inner, w_in, NOISE_FLOOR)
    mask = (tophat >= denom * MIN_SCR).astype(np.uint8)

    whole, _ = bench(detect, frame, reps)
    items = [
        ("MORPH_TOPHAT ellipse %d" % SE_SIZE,
         lambda a: cv2.morphologyEx(a, cv2.MORPH_TOPHAT, se)),
        ("  .astype(float32)", lambda a: cv2.morphologyEx(a, cv2.MORPH_TOPHAT, se).astype(np.float32)),
        ("blur %dx%d (inner)" % (SE_SIZE, SE_SIZE), lambda a: cv2.blur(tophat, (SE_SIZE, SE_SIZE))),
        ("blur %dx%d (outer)" % (OUTER_SIZE, OUTER_SIZE), lambda a: cv2.blur(tophat, (OUTER_SIZE, OUTER_SIZE))),
        ("addWeighted -> denom", lambda a: cv2.addWeighted(outer, w_out, inner, w_in, NOISE_FLOOR)),
        ("mask = tophat >= denom*MIN_SCR", lambda a: (tophat >= denom * MIN_SCR).astype(np.uint8)),
        ("connectedComponentsWithStats", lambda a: cv2.connectedComponentsWithStats(mask, connectivity=8)),
    ]
    print("  %-38s %8s   %6s" % ("stage", "time", "share"))
    for label, fn in items:
        m, _ = bench(fn, frame, reps)
        line(label, m, whole)
    print("  " + "-" * 56)
    line("detect() whole", whole)
    return whole


def se_scan(frame, reps):
    print("  cost vs DETECTOR_SE_SIZE (currently %d). The outer box is 3x this, so" % SE_SIZE)
    print("  both the morphology and the annulus scale with it:")
    print("  %-6s %-10s %10s   %s" % ("SE", "SE area", "tophat", "per px*element"))
    for S in (15, 27, 35, 45, 55, 75):
        se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (S, S))
        m, _ = bench(lambda a: cv2.morphologyEx(a, cv2.MORPH_TOPHAT, se), frame, reps)
        area = int(se.sum())
        print("  %-6d %-10d %8.1f ms   %.2e%s"
              % (S, area, 1000 * m, m / (frame.size * area),
                 "   <- config" if S == SE_SIZE else ""))
    print("\n  Cost is linear in pixels x SE area and OpenCV has no ellipse")
    print("  decomposition, so it evaluates every element at every pixel. SE_SIZE is")
    print("  therefore a QUADRATIC cost knob, not just a detection knob.")


def _pipeline(frame, se, se_size, dtype_u8=True):
    """detect(), parameterised on the SE and whether morphology runs in the frame dtype."""
    outer_size = 3 * se_size
    if dtype_u8:
        tophat = cv2.morphologyEx(frame, cv2.MORPH_TOPHAT, se).astype(np.float32)
    else:
        img = frame.astype(np.float32)
        tophat = np.clip(img - cv2.morphologyEx(img, cv2.MORPH_OPEN, se), 0, None)
    ia, oa = se_size * se_size, outer_size * outer_size
    inner = cv2.blur(tophat, (se_size, se_size))
    outer = cv2.blur(tophat, (outer_size, outer_size))
    denom = cv2.addWeighted(outer, oa / (oa - ia), inner, -ia / (oa - ia), NOISE_FLOOR)
    mask = (tophat >= denom * MIN_SCR).astype(np.uint8)
    n, _, st, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = st[i]
        if MIN_AREA <= area <= MAX_AREA:
            out.append((x, y, w, h, float((tophat[y:y+h, x:x+w] / denom[y:y+h, x:x+w]).max())))
    return out


def variants(frames, reps):
    S = SE_SIZE
    ell = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (S, S))
    rect = cv2.getStructuringElement(cv2.MORPH_RECT, (S, S))
    half = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (S // 2 | 1, S // 2 | 1))

    def dec2(frame):
        boxes = _pipeline(frame[::2, ::2], half, S // 2 | 1)
        return [(x * 2, y * 2, w * 2, h * 2, s) for (x, y, w, h, s) in boxes]

    cands = [
        ("float32 + ellipse (pre-2026-09 form)", lambda f: _pipeline(f, ell, S, dtype_u8=False)),
        ("MORPH_RECT instead of ellipse", lambda f: _pipeline(f, rect, S)),
        ("decimate 2x, SE %d" % (S // 2 | 1), dec2),
    ]
    base_t, _ = bench(detect, frames[0], reps)
    print("  baseline: detect() as shipped = %.1f ms\n" % (1000 * base_t))
    print("  %-34s %9s %7s  %-13s %s"
          % ("variant", "time", "speedup", "exact match", "detections kept"))
    for label, fn in cands:
        m, _ = bench(fn, frames[0], reps)
        same, kept, base_n, var_n = 0, 0, 0, 0
        for f in frames:
            a, b = detect(f), fn(f)
            if len(a) == len(b) and all(x[:4] == y[:4] for x, y in zip(a, b)):
                same += 1
            k, na, nb = _recall(a, b, tol=4.0)
            kept += k
            base_n += na
            var_n += nb
        exact = "%d/%d frames" % (same, len(frames))
        # Strict geometry is too harsh on a decimated variant -- it can only emit
        # even coordinates -- so also report how many of the real detector's
        # detections the variant reproduces within a few pixels.
        recall = ("%d/%d (%.0f%%), emitted %d"
                  % (kept, base_n, 100.0 * kept / max(base_n, 1), var_n))
        print("  %-34s %7.1f ms %6.2fx  %-13s %s" % (label, 1000 * m, base_t / m, exact, recall))
    print("\n  'detections kept' counts baseline detections a variant reproduces with its")
    print("  centre within 4 px. A variant that keeps only ~70% is a DIFFERENT detector,")
    print("  not a faster one -- it may still be the right trade, but deciding that needs")
    print("  footage with a real target in it. On a bench scene almost every detection is")
    print("  clutter, so churn here says nothing about whether a drone survives.")


def _recall(base, var, tol):
    """(matched, n_base, n_var) by nearest-centre, greedily, within tol pixels."""
    cb = [(x + w / 2.0, y + h / 2.0) for x, y, w, h, *_ in base]
    cv_ = [(x + w / 2.0, y + h / 2.0) for x, y, w, h, *_ in var]
    used, matched = set(), 0
    for p in cb:
        best, bi = None, None
        for i, q in enumerate(cv_):
            if i in used:
                continue
            d = ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5
            if best is None or d < best:
                best, bi = d, i
        if bi is not None and best <= tol:
            matched += 1
            used.add(bi)
    return matched, len(cb), len(cv_)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/thermal0")
    ap.add_argument("--npy", help="profile a saved (n, h, w) uint8 stack instead")
    ap.add_argument("--frames", type=int, default=12, help="frames to grab (default 12)")
    ap.add_argument("--reps", type=int, default=4, help="timed repeats per stage (default 4)")
    ap.add_argument("--roi", metavar="WxH",
                    help="crop each frame to this size first, e.g. 520x300 -- the size "
                         "tools/live_track.py actually hands the detector once locked")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=1024)
    ap.add_argument("--se-scan", action="store_true", help="also scan cost vs SE size")
    ap.add_argument("--variants", action="store_true", help="also price the alternatives")
    ap.add_argument("--save", metavar="FILE.npy", help="save the grabbed frames")
    args = ap.parse_args()

    if args.npy:
        frames = np.load(args.npy)
    else:
        print("grabbing %d frames from %s ..." % (args.frames, args.device))
        frames = grab(args.device, args.frames, args.width, args.height)
    if args.save:
        np.save(args.save, frames)
        print("saved %s" % args.save)

    if args.roi:
        rw, rh = (int(v) for v in args.roi.lower().split("x"))
        h, w = frames.shape[1:3]
        y0, x0 = max(0, (h - rh) // 2), max(0, (w - rw) // 2)
        frames = frames[:, y0:y0 + rh, x0:x0 + rw].copy()

    print("\nopencv %s, %d threads of %d cpus" % (cv2.__version__, cv2.getNumThreads(),
                                                  cv2.getNumberOfCPUs()))
    print("frames %d x %dx%d %s   SE=%d OUTER=%d MIN_SCR=%.1f"
          % (len(frames), frames.shape[2], frames.shape[1], frames.dtype,
             SE_SIZE, OUTER_SIZE, MIN_SCR))
    nd = [len(detect(f)) for f in frames]
    print("detections/frame: min %d med %d max %d" % (min(nd), sorted(nd)[len(nd)//2], max(nd)))
    print("\n== stage breakdown")
    whole = stages(frames[0], args.reps)
    budget = 1.0 / 25.0
    print("\n  budget at 25 fps is %.1f ms -> this is %.1fx over on a %dx%d input"
          % (1000 * budget, whole / budget, frames.shape[2], frames.shape[1]))
    if args.se_scan:
        print("\n== SE size scan")
        se_scan(frames[0], args.reps)
    if args.variants:
        print("\n== variants")
        variants(frames, args.reps)


if __name__ == "__main__":
    main()
