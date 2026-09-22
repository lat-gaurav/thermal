#!/usr/bin/env python3
"""Was any frame dropped while recording, and by which layer?

    tools/drop_report.py logs/rawrec/flight-S-01.rawrec
    tools/drop_report.py logs/rawrec/*.rawrec              # whole directory, shell glob
    tools/drop_report.py FILE.rawrec --csv logs/telemetry/los-S.csv

A .rawrec has no index or trailer (flight/rawrec.py) -- there is nothing in
the file that says "this many frames should be here". The only way to know a
frame is missing is to notice the CLOCK skipped: every frame's own t_mono is
in the file header, so a t_mono step much larger than the file's own median
inter-frame interval means real frames never arrived (dropped by
flight.telemetry/flight.rawrec's bounded queues, or -- if the camera itself
stalled -- never grabbed at all).

TWO INDEPENDENT SIGNALS, cross-checked, because either alone can mislead:

  idx gap      a t_mono step > 1.5x the median dt, from the .rawrec's own
              frame index (built once, then cached in a .rawrec.idx sidecar
              by tools/rawrec_viewer.py -- see that module's build_index()).
              This is ground truth for "how many frames are actually in the
              file", independent of anything the pipeline itself believed.
  raw_dropped  the RawRecWriter's own cumulative drop counter for this
              episode, read from the matching los-*.csv (found the same way
              every viewer finds it -- experiment/los_static_track.py's
              find_los_csv()). This is what the RECORDER believed at the
              time, and is what tells you it was the disk queue overflowing
              specifically, not e.g. the camera itself stalling (grabbed
              would show that instead -- see below).

The two should roughly agree in total. When they diverge by a lot, that is
itself informative: a large idx gap with a SMALL raw_dropped delta over the
same window means the camera's own grab thread stalled (check `grabbed` in
the csv -- flight/camera.py's src.grabbed, which should keep climbing at a
steady rate through any real recorder-only drop).

WHAT THIS DOES NOT TELL YOU ON ITS OWN: whether detection/tracking/uplink
were ALSO silent during a gap, not just recording. Pass --csv-activity (or
just open the file's csv directly) to see whether any csv rows were logged
inside a gap window -- zero means the whole processing loop stalled too, not
only the recorder. This distinction is exactly what surfaced
flight/telemetry.py's TelemetryWriter: on 2026-09-18, most of the gap time in
one sortie had zero csv rows at all, because the synchronous csv_f.flush()
that used to sit in the main loop was blocking on the very same saturated
disk RawRecWriter was already struggling with.
"""
import argparse
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve().parent
_ROOT = _HERE.parent
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_HERE))

import numpy as np

import rawrec_viewer as rv          # noqa: E402  read_header/build_index
from experiment.los_static_track import find_los_csv    # noqa: E402


def gap_scan(path, thresh_mult=1.5):
    """(frames, fps, span_s, gaps) -- gaps is [(t0, t1, n_missing), ...]."""
    meta = rv.read_header(path)
    frames = rv.build_index(path, meta)
    t = np.array([tm for _, tm in frames])
    dt = np.diff(t)
    med = float(np.median(dt)) if len(dt) else 0.0
    fps = 1.0 / med if med > 0 else float("nan")
    gaps = []
    if med > 0:
        thresh = med * thresh_mult
        for i in np.where(dt > thresh)[0]:
            n_missing = max(0, round(dt[i] / med) - 1)
            gaps.append((float(t[i]), float(t[i + 1]), n_missing))
    span = float(t[-1] - t[0]) if len(t) else 0.0
    return {"n_frames": len(frames), "fps": fps, "span_s": span, "gaps": gaps}


def episode_num(rawrec_path):
    """The integer episode this file's '-NN' suffix encodes, or None (no suffix)."""
    stem = pathlib.Path(rawrec_path).stem
    tail = stem.rsplit("-", 1)[-1]
    return int(tail) if tail.isdigit() else None


def csv_cross_check(rawrec_path, csv_override=None):
    """(csv_path, raw_dropped_total, n_rows_in_episode) or (None, None, None).

    raw_dropped is cumulative PER EPISODE, counting from 0 the instant that
    episode's RawRecWriter opens (see flight/rawrec.py) -- so the total for
    the episode is simply the LARGEST value logged for it, never a delta
    across rows. A delta would be wrong whenever a drop happens before the
    episode's first logged csv row (e.g. right at open): every subsequent row
    then carries the same already-nonzero count, and max-min silently reads
    as zero. (Caught by cross-checking this function's output against a
    by-hand gap scan on flight-20260916-110640-538d6dbe-01.rawrec, whose 1621
    rows all read exactly 13 -- one early drop, then none for the rest of the
    episode.)

    The csv's own 'episode' column is the exact integer the pipeline passed
    to open_episode(), which is exactly the '-NN' suffix on this file's name,
    so the match is direct: no off-by-one, and no guessing.
    """
    csv_path = pathlib.Path(csv_override) if csv_override else find_los_csv(rawrec_path)
    if csv_path is None or not pathlib.Path(csv_path).is_file():
        return None, None, None
    import csv as csvmod
    ep = episode_num(rawrec_path)
    rows = [r for r in csvmod.DictReader(open(csv_path))
            if ep is None or r.get("episode") == str(ep)]
    dropped = [int(r["raw_dropped"]) for r in rows
               if r.get("raw_dropped", "").strip().lstrip("-").isdigit()]
    if not dropped:
        return csv_path, None, len(rows)
    return csv_path, max(dropped), len(rows)


def report_one(path, csv_override, thresh_mult):
    g = gap_scan(path, thresh_mult)
    csv_path, raw_dropped, n_csv_rows = csv_cross_check(path, csv_override)
    n_written = g["n_frames"]
    n_missing = sum(m for _, _, m in g["gaps"])
    denom = n_written + (raw_dropped or 0)
    pct = 100.0 * (raw_dropped or 0) / denom if denom else 0.0
    return {
        "file": pathlib.Path(path).name, "n_written": n_written,
        "fps": g["fps"], "span_s": g["span_s"], "n_gaps": len(g["gaps"]),
        "idx_missing_est": n_missing,
        "worst_gap_s": max((t1 - t0 for t0, t1, _ in g["gaps"]), default=0.0),
        "csv_path": csv_path.name if csv_path else None,
        "raw_dropped": raw_dropped, "drop_pct": pct, "gaps": g["gaps"],
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rawrec", nargs="+", help="one or more .rawrec files")
    ap.add_argument("--csv", help="override auto-discovery (only valid for one file)")
    ap.add_argument("--thresh", type=float, default=1.5,
                     help="flag a gap when dt exceeds this x the median (default %(default)s)")
    ap.add_argument("--list-gaps", action="store_true",
                     help="print every individual gap, not just the per-file summary")
    args = ap.parse_args()
    if args.csv and len(args.rawrec) != 1:
        sys.exit("--csv only makes sense with exactly one .rawrec")

    rows = []
    for p in args.rawrec:
        p = pathlib.Path(p)
        if not p.is_file():
            print("%s: no such file, skipping" % p, file=sys.stderr)
            continue
        r = report_one(p, args.csv, args.thresh)
        rows.append(r)

    if not rows:
        sys.exit("nothing to report")

    hdr = ("%-46s %8s %6s %8s %5s %10s %8s %14s %9s" %
           ("file", "written", "fps", "span_s", "gaps", "idx_miss", "worst_s",
            "raw_dropped", "drop%"))
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        dropped_str = "n/a" if r["raw_dropped"] is None else str(r["raw_dropped"])
        # "n/a" means unconfirmed (no matching csv), not "confirmed 0" -- keep
        # those visually distinct so a scan of this column can't mistake one
        # for the other.
        pct_str = "n/a" if r["raw_dropped"] is None else "%.2f%%" % r["drop_pct"]
        print("%-46s %8d %6.2f %8.1f %5d %10d %8.2f %14s %9s" %
              (r["file"], r["n_written"], r["fps"], r["span_s"], r["n_gaps"],
               r["idx_missing_est"], r["worst_gap_s"], dropped_str, pct_str))
        if args.list_gaps:
            for t0, t1, n in r["gaps"]:
                print("      gap  t=%.3f -> %.3f  (%.2fs, ~%d frames)" % (t0, t1, t1 - t0, n))

    n_files = len(rows)
    total_written = sum(r["n_written"] for r in rows)
    n_unconfirmed = sum(1 for r in rows if r["raw_dropped"] is None)
    total_dropped = sum(r["raw_dropped"] for r in rows if r["raw_dropped"] is not None)
    flagged = [r for r in rows if (r["raw_dropped"] or 0) > 0 or r["n_gaps"] > 0]
    print()
    print("%d file(s), %d frames written, %d frames dropped (csv-confirmed)%s"
          % (n_files, total_written, total_dropped,
             "  -- %d file(s) had no matching csv, not counted here" % n_unconfirmed
             if n_unconfirmed else ""))
    if flagged:
        print("%d/%d file(s) show any drop at all: %s"
              % (len(flagged), n_files, ", ".join(r["file"] for r in flagged)))
    else:
        print("no drops found in any file checked")


if __name__ == "__main__":
    main()
