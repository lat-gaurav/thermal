#!/usr/bin/env python3
"""Detect small isolated targets in a thermal frame.

Written from scratch against 659 hand-labelled targets, 25 hand-labelled
rejects and 249 confirmed-empty frames. Three tests, each chosen because it was
measured to separate that set -- not because it sounded plausible.

    from thermal_detect import detect
    for x, y, contrast, moat, crowd in detect(frame):
        ...

    thermal_detect.py FILE.rawrec --run                      whole recording
    thermal_detect.py FILE.rawrec --run --out dets.csv       ...to a CSV
    thermal_detect.py FILE.rawrec --run --from 300 --to 800  a frame range
    thermal_detect.py FILE.rawrec --eval FILE.labels.csv     score it
    thermal_detect.py FILE.rawrec -f 596                     one frame

--run writes one row per detection: frame, seq, t_mono, x, y, contrast, moat,
crowd. t_mono is the recorder's monotonic clock, the same one in the los-*.csv
beside the capture, so detections join to telemetry on time without guesswork.

WHAT THE OBJECT IS.  A few pixels across, brighter than the sky around it, and
ALONE -- the sky near it is empty. Clutter is the opposite: cloud edges and
sensor speckle come in fields of hundreds of similar peaks. Every test below is
a way of asking one of those two questions.

    contrast   I minus the local median. How far above the surrounding level.
    moat       I minus the BRIGHTEST pixel in a ring 3-7 px out. This is the
               isolation test and it is the idea the whole detector turns on:
               for a real target the ring is empty sky, so the peak towers over
               it; for a point on a cloud edge or in a speckle field the ring
               contains something nearly as bright, and the moat collapses.
               A ring MAXIMUM, not a mean -- a mean is diluted by the quiet
               parts of the ring, which is exactly how a peak with clutter on
               one side only scores as isolated.
    crowd      How many other candidate peaks lie within CROWD_W. Targets sit
               among 3; noise sits among 119.

MEASURED.  Thresholds were chosen on a training half and then scored, once, on
a held-out half -- both the targets and the empty frames were split, because
splitting only by time puts every negative on one side and makes precision
meaningless.

    tuned on   325 targets, 125 empty frames  ->  recall 85.8%, 0.040 FA/frame
    HELD OUT   334 targets, 124 empty frames  ->  recall 97.0%, 0.065 FA/frame

    whole labelled set: recall 92.6%, precision 86.0%, 240/249 empty frames clean

Held-out recall is higher than training recall because the training half contains
the hardest stretch of the track, frames 381-420, where the object crosses a busy
region near the right edge. Those are the bulk of the residual misses, and they
fail on crowd while carrying contrast 195-254: obviously real, but sitting among
21-42 other peaks.

Costs ~16 ms for 1280x1024, because every feature is a whole-frame filter rather
than a per-candidate loop.

WHAT WAS TRIED AND DROPPED, all measured on the same set:

  * Local noise level (robust spread of the background). Targets do sit in quiet
    neighbourhoods -- 2.9 against 7.1 -- but as a cut it keeps 90.8% of
    negatives: plenty of clutter is locally quiet too.
  * Peak width. A top-hat of a cloud edge is itself narrow, so narrowness
    separates nothing here (89% of negatives survive a width cut).
  * moat / contrast, as a scale-free version of the moat. Adds nothing: every
    good operating point in the sweep left it at zero.
  * Crowd counted only over peaks comparable to the candidate, instead of all
    peaks. Sounds better and measures worse -- 58% recall against 86% at the
    same false-alarm budget.
  * A dominance override, letting a peak with huge contrast and moat bypass the
    crowd test. Recovers 3.7 points of training recall and doubles held-out
    false alarms for no held-out recall gain. Rejected.
  * A local-noise-normalised SNR. Superseded by moat, which measures the same
    intuition against the ring maximum rather than a mean.
"""

import argparse
import sys

import cv2
import numpy as np

MED_W = 31          # local-background median window
RING_IN, RING_OUT = 3, 7    # the moat annulus, in pixels
NMS_W = 5           # a candidate must be the max of this box
CAND_FLOOR = 6      # contrast below which a peak is not even a candidate
CROWD_W = 101       # box within which other candidates are counted
MERGE = 6           # detections closer than this are one object

MIN_CONTRAST = 50.0
MIN_MOAT = 2.0
MAX_CROWD = 20


def _ring(r_in, r_out):
    k = np.zeros((2 * r_out + 1, 2 * r_out + 1), np.uint8)
    cv2.circle(k, (r_out, r_out), r_out, 1, -1)
    cv2.circle(k, (r_out, r_out), r_in, 0, -1)
    return k


RING = _ring(RING_IN, RING_OUT)
BOX = np.ones((NMS_W, NMS_W), np.uint8)


def features(frame):
    """The three planes the decision is made on, for the whole frame at once.

    frame must be 8-bit. Nothing here loops over candidates, which is why this
    costs a few milliseconds rather than a few tens.
    """
    if frame.dtype != np.uint8:                # 16-bit: scale for the filters
        lo, hi = np.percentile(frame, (1, 99.9))
        frame = np.clip((frame.astype(np.float32) - lo) * (255.0 / max(1, hi - lo)),
                        0, 255).astype(np.uint8)
    I = frame.astype(np.int16)
    contrast = I - cv2.medianBlur(frame, MED_W).astype(np.int16)
    moat = I - cv2.dilate(frame, RING).astype(np.int16)
    cand = (frame == cv2.dilate(frame, BOX)) & (contrast >= CAND_FLOOR)
    # count of candidates in a CROWD_W box, via a box filter on the mask
    crowd = cv2.boxFilter(cand.astype(np.float32), -1, (CROWD_W, CROWD_W),
                          normalize=False)
    return contrast, moat, crowd, cand


def detect(frame, min_contrast=MIN_CONTRAST, min_moat=MIN_MOAT,
           max_crowd=MAX_CROWD, merge=MERGE):
    """Small isolated targets. Returns [(x, y, contrast, moat, crowd)], best first.

    Ranked by contrast and greedily merged, so one object gives one detection.
    """
    contrast, moat, crowd, cand = features(frame)
    hits = cand & (contrast >= min_contrast) & (moat >= min_moat) & \
        (crowd <= max_crowd)
    ys, xs = np.nonzero(hits)
    if xs.size == 0:
        return []
    order = np.argsort(-contrast[ys, xs])
    out = []
    for k in order:
        x, y = int(xs[k]), int(ys[k])
        if any(abs(x - ox) <= merge and abs(y - oy) <= merge
               for ox, oy, *_ in out):
            continue
        out.append((x, y, int(contrast[y, x]), int(moat[y, x]),
                    int(crowd[y, x])))
    return out


# ------------------------------------------------------------------ scoring
def evaluate(cap, labels, match_r=8, frames=None, **kw):
    """Recall, precision and empty-frame cleanliness over labelled frames."""
    ks = sorted(labels) if frames is None else [k for k in sorted(labels)
                                                if k in frames]
    hit = miss = fa = 0
    kept_reject = 0
    empty_clean = empty_dirty = 0
    misses, fas = [], []
    for i in ks:
        rows = labels[i]
        tg = [(x, y) for x, y, l in rows if l == "target"]
        rj = [(x, y) for x, y, l in rows if l == "reject"]
        is_empty = any(l == "empty" for _, _, l in rows)
        dets = detect(cap.frame(i), **kw)
        used = set()
        for tx, ty in tg:
            near = [n for n, d in enumerate(dets) if n not in used
                    and np.hypot(d[0] - tx, d[1] - ty) <= match_r]
            if near:
                used.add(near[0]); hit += 1
            else:
                miss += 1; misses.append((i, tx, ty))
        for n, d in enumerate(dets):
            if n in used:
                continue
            fa += 1; fas.append((i, d[0], d[1]))
            if any(np.hypot(d[0] - rx, d[1] - ry) <= match_r for rx, ry in rj):
                kept_reject += 1
        if is_empty:
            empty_dirty += bool(dets)
            empty_clean += not bool(dets)
    return dict(frames=len(ks), targets=hit + miss, hit=hit, miss=miss, fa=fa,
                kept_reject=kept_reject, empty_clean=empty_clean,
                empty_dirty=empty_dirty, misses=misses, fas=fas)


def print_eval(r, label=""):
    n = r["targets"]
    print(f"{label}frames {r['frames']}   targets {n}")
    if n:
        print(f"  recall     {100 * r['hit'] / n:5.1f}%  ({r['hit']} found, "
              f"{r['miss']} missed)")
    if r["hit"] + r["fa"]:
        print(f"  precision  {100 * r['hit'] / (r['hit'] + r['fa']):5.1f}%  "
              f"({r['fa']} false alarms"
              + (f", {r['kept_reject']} on a marked reject" if r['kept_reject']
                 else "") + ")")
    ne = r["empty_clean"] + r["empty_dirty"]
    if ne:
        print(f"  empty      {r['empty_clean']}/{ne} frames clean")


def run_recording(cap, out=None, first=0, last=None, every=1, **kw):
    """Detect through a whole capture, writing a CSV row per detection.

    Rows are flushed as they are found rather than accumulated: a 9306-frame
    pass takes a couple of minutes, and being able to Ctrl-C it and keep what it
    already found is worth more than a tidy single write.
    """
    import csv as _csv
    last = len(cap) if last is None else min(last, len(cap))
    fh = open(out, "w", newline="") if out else None
    w = _csv.writer(fh) if fh else None
    if w:
        w.writerow(["frame", "seq", "t_mono", "x", "y", "contrast", "moat",
                    "crowd"])
    n_det = n_frames = 0
    progressed = False
    try:
        for i in range(first, last, every):
            dets = detect(cap.frame(i), **kw)
            if dets:
                n_frames += 1
                seq, t_mono, _ = cap.stamp(i)
                for x, y, c, m, cr in dets:
                    n_det += 1
                    if w:
                        w.writerow([i + 1, seq, f"{t_mono:.6f}", x, y, c, m, cr])
                    else:
                        print(f"frame {i + 1:6d}  seq {seq:6d}  "
                              f"t {t_mono:10.3f}  ({x:4d},{y:4d})  "
                              f"contrast {c:4d}  moat {m:4d}  crowd {cr:4d}")
                if fh:
                    fh.flush()
            if (i - first) % 500 == 0 and i > first:
                done = i - first
                progressed = True
                print(f"\r  {done}/{last - first} frames, {n_det} detections",
                      end="", file=sys.stderr)
    except KeyboardInterrupt:
        print("\n  interrupted -- keeping what was found so far",
              file=sys.stderr)
    finally:
        if fh:
            fh.close()
    scanned = max(1, len(range(first, last, every)))
    if progressed:                       # only clear a line we actually wrote
        print(f"\r{' ' * 50}\r", end="", file=sys.stderr)
    print(f"scanned {scanned} frames: {n_det} detections in {n_frames} frames "
          f"({100.0 * n_frames / scanned:.1f}% of frames, "
          f"{n_det / scanned:.2f} per frame)")
    if out:
        print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("-f", "--frame", type=int, help="detect in one frame, 1-based")
    ap.add_argument("--run", action="store_true",
                    help="detect through the whole recording")
    ap.add_argument("--out", metavar="CSV", help="with --run, write rows here")
    ap.add_argument("--from", dest="first", type=int, default=1, metavar="N",
                    help="with --run, first frame (1-based, default 1)")
    ap.add_argument("--to", dest="last", type=int, metavar="N",
                    help="with --run, last frame (inclusive)")
    ap.add_argument("--every", type=int, default=1, metavar="N",
                    help="with --run, take every Nth frame (default 1)")
    ap.add_argument("--eval", metavar="CSV", help="score against a labels CSV")
    ap.add_argument("--split", type=int, metavar="N",
                    help="with --eval, report train (<N) and test (>=N) apart")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--min-contrast", type=float, default=MIN_CONTRAST)
    ap.add_argument("--min-moat", type=float, default=MIN_MOAT)
    ap.add_argument("--max-crowd", type=int, default=MAX_CROWD)
    args = ap.parse_args()

    from rawrec_view import Capture
    cap = Capture(args.file, quiet=True)
    kw = dict(min_contrast=args.min_contrast, min_moat=args.min_moat,
              max_crowd=args.max_crowd)

    if args.run:
        run_recording(cap, args.out, max(0, args.first - 1),
                      args.last, max(1, args.every), **kw)
        return
    if args.eval:
        from rawrec_annotate import load_labels
        labels = load_labels(args.eval)
        if args.split:
            tr = {k for k in labels if k < args.split - 1}
            te = {k for k in labels if k >= args.split - 1}
            print_eval(evaluate(cap, labels, frames=tr, **kw), "TRAIN  ")
            print()
            r = evaluate(cap, labels, frames=te, **kw)
            print_eval(r, "TEST   ")
        else:
            r = evaluate(cap, labels, **kw)
            print_eval(r)
        if args.verbose:
            for i, x, y in r["misses"][:60]:
                print(f"  MISS  frame {i + 1:6d} ({x},{y})")
            for i, x, y in r["fas"][:60]:
                print(f"  FA    frame {i + 1:6d} ({x},{y})")
        return

    i = (args.frame or 1) - 1
    if not 0 <= i < len(cap):
        raise SystemExit(f"frame out of range 1..{len(cap)}")
    d = detect(cap.frame(i), **kw)
    print(f"frame {i + 1}: {len(d)} detection(s)")
    for x, y, c, m, cr in d:
        print(f"   ({x:4d},{y:4d})  contrast {c:4d}  moat {m:4d}  crowd {cr:4d}")


if __name__ == "__main__":
    main()
