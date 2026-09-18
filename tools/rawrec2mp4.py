#!/usr/bin/env python3
"""Convert a .rawrec capture to an H.264 mp4 for human review.

    rawrec2mp4.py logs/flight-20260915-132340-26fd36c7-01.rawrec
    rawrec2mp4.py flight.rawrec -o out.mp4 --crf 20 --scale 0.5
    rawrec2mp4.py flight.rawrec --start 30 --end 90     seconds into the recording
    rawrec2mp4.py flight.rawrec --as-captured --no-overlay   frames back to back, clean

The .rawrec IS the archive: it is the format every reader in this repo already
opens, it survives a crash because there is no index to finalise, and it holds
the exact bytes the sensor sent. An mp4 is a LOSSY DERIVATIVE for watching and
for sending to people who do not have this repo. Never convert and delete --
`tools/det_bench.py`, `tools/rawrec_viewer.py` and `experiment/los_static_track.py`
all need the original, and H.264 has already destroyed the per-pixel values the
detector thresholds on.

THE OUTPUT IS ON A REAL TIMEBASE, not a frame counter. Two separate reasons:

  1. `advertised_fps` IN THE HEADER LIES. It is whatever the core was asked for,
     and the core does not honour it: a file recorded on 2026-09-08 carries
     advertised_fps 50.0 and actually contains 25 fps. So the rate comes from
     the median of the recorded t_mono values, exactly as tools/rawrec_viewer.py
     derives it, and never from the header.

  2. FRAMES GO MISSING, and a video that ignores that silently speeds up. The
     recorder drops rather than blocks when the disk falls behind (flight/rawrec.py),
     so a capture can contain a real gap. By default each output frame is placed
     at its true t_mono on a fixed grid and gaps are filled by holding the last
     frame, so ELAPSED VIDEO TIME EQUALS ELAPSED RECORDING TIME. A 2 s dropout
     looks like a 2 s freeze, which is the truth, instead of vanishing. Pass
     --as-captured for the other behaviour, which is only right when you want to
     inspect consecutive frames rather than watch a sortie.

THE OVERLAY IS THE JOIN KEY, which is why it is on by default. los-*.csv carries
a `rec_idx` column precisely so a CSV row can be mapped onto the recorded video;
without the index burned in there is no way to do that from a player's scrub bar.
--no-overlay gives clean pixels.

Pi 5 has NO hardware H.264 encoder, so this is libx264 on the CPU: expect
roughly real time at 1280x1024 on `veryfast`. It is not something to run during
a sortie.
"""
import argparse
import os
import pathlib
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

import rawrec_viewer as rv          # noqa: E402  read_header/build_index/measured_fps
import telemetry_viewer as tv       # noqa: E402  --telemetry: annotate_from_row + the
                                    # same csv join telemetry_viewer.py itself uses


def stretch_params(path, meta, frames, n_sample=60):
    """Global 1-99 percentile window for 16-bit captures, or None for 8-bit.

    GLOBAL, not per frame. rv.to_display() stretches each frame against its own
    percentiles, which is right for a viewer that shows one frame at a time and
    wrong for video: the window then moves whenever the scene does, and the
    result flickers on every cut. One window sampled across the file keeps the
    brightness of a target comparable from second to second, which is the whole
    reason for looking at it.
    """
    if meta["bpp"] == 1:
        return None
    idx = np.linspace(0, len(frames) - 1, min(n_sample, len(frames))).astype(int)
    los, his = [], []
    with open(path, "rb") as f:
        for i in idx:
            f.seek(frames[i][0])
            buf = f.read(meta["frame_bytes"])
            a = np.frombuffer(buf, dtype=np.uint16)
            lo, hi = np.percentile(a, (1, 99))
            los.append(lo)
            his.append(hi)
    lo, hi = float(np.median(los)), float(np.median(his))
    return (lo, hi if hi > lo else lo + 1.0)


def to8(buf, meta, stretch):
    """One record's pixels -> an 8-bit HxW array ready to encode."""
    h, w = meta["height"], meta["width"]
    if meta["bpp"] == 1:
        return np.frombuffer(buf, dtype=np.uint8).reshape(h, w)
    a = np.frombuffer(buf, dtype=np.uint16).reshape(h, w).astype(np.float32)
    lo, hi = stretch
    return np.clip((a - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)


def draw_overlay(img, rec_idx, t_rel, t_mono, held, mag=1.0):
    """Burn the join key in. Mutates and returns img (8-bit, 1 or 3 channel).

    mag PRE-COMPENSATES FOR --scale. The text is drawn at capture resolution and
    ffmpeg downscales the whole frame afterwards, so at --scale 0.5 a fixed font
    arrives half size and stops being readable at exactly the moment the frame
    got smaller. Drawing it 1/scale larger lands it the same size on the output.
    """
    txt = "rec_idx %d   t+%7.3fs   t_mono %.3f" % (rec_idx, t_rel, t_mono)
    if held:
        txt += "   [HELD: gap in recording]"
    fs = 0.6 * mag
    # A plain int colour is cv::Scalar(v, 0, 0, 0) on a 3-channel image --
    # pure blue, not white. Replicate across channels so --telemetry's BGR
    # frames still get an actual black/white overlay, not a blue one.
    is_colour = img.ndim == 3
    # Drawn twice: black underlay then white, so it stays readable over both a
    # hot sky and a cold one. A filled box would hide pixels that matter.
    for v, thick in ((0, max(2, int(round(4 * mag)))), (255, max(1, int(round(mag))))):
        colour = (v, v, v) if is_colour else v
        cv2.putText(img, txt, (int(12 * mag), img.shape[0] - int(14 * mag)),
                    cv2.FONT_HERSHEY_SIMPLEX, fs, colour, thick, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rawrec")
    ap.add_argument("-o", "--out", help="default: alongside the input, .mp4")
    ap.add_argument("--fps", type=float, default=None,
                    help="override the rate measured from the timestamps")
    ap.add_argument("--crf", type=int, default=23,
                    help="x264 quality, lower is better; 18 is near-transparent "
                         "(default %(default)s)")
    ap.add_argument("--preset", default="veryfast",
                    help="x264 speed/size tradeoff (default %(default)s)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="output scale factor, e.g. 0.5 (default %(default)s)")
    ap.add_argument("--start", type=float, default=0.0, metavar="S",
                    help="seconds into the recording to start")
    ap.add_argument("--end", type=float, default=None, metavar="S",
                    help="seconds into the recording to stop")
    ap.add_argument("--as-captured", action="store_true",
                    help="emit frames back to back, ignoring the timestamps")
    ap.add_argument("--no-overlay", action="store_true",
                    help="clean pixels: no rec_idx/time burn-in")
    ap.add_argument("--telemetry", action="store_true",
                    help="also burn in the logged CUBE-valid banner, ROI box, "
                         "LOS marker and cue -- the same annotate_from_row() "
                         "tools/telemetry_viewer.py draws, from that flight's "
                         "own los-*.csv. Needs a matching csv (auto-found, or "
                         "pass --csv). Output becomes colour, not grey.")
    ap.add_argument("--csv", help="override: path to the matching los-*.csv "
                                  "(only used with --telemetry)")
    args = ap.parse_args()

    ff = shutil.which("ffmpeg")
    if not ff:
        sys.exit("ffmpeg is not installed:  sudo apt install ffmpeg")
    src = pathlib.Path(args.rawrec)
    if not src.is_file():
        sys.exit("%s: no such file" % src)
    out = pathlib.Path(args.out) if args.out else src.with_suffix(".mp4")

    meta = rv.read_header(src)
    frames = rv.build_index(src, meta)
    fps_meas = rv.measured_fps(frames)
    fps = args.fps or fps_meas
    W, H, fb = meta["width"], meta["height"], meta["frame_bytes"]

    t0 = frames[0][1]
    span = frames[-1][1] - t0

    per_frame = None
    if args.telemetry:
        los_track = rv._load_los_track()
        csv_path = pathlib.Path(args.csv) if args.csv else los_track.find_los_csv(src)
        if csv_path is None or not pathlib.Path(csv_path).is_file():
            sys.exit("%s: no matching los-*.csv found for --telemetry "
                      "(pass --csv to point at one)" % src)
        rows, _fields = tv.load_telemetry(csv_path)
        if not rows:
            sys.exit("%s: no rows with a usable t_mono" % csv_path)
        per_frame = tv.join_by_t_mono([t for _, t in frames], rows)

    # yuv420p needs even dimensions, and an odd one makes ffmpeg fail after the
    # whole read has already happened.
    ow, oh = W, H
    if args.scale != 1.0:
        ow, oh = int(round(W * args.scale)) & ~1, int(round(H * args.scale)) & ~1
        if ow < 2 or oh < 2:
            sys.exit("--scale %.3f leaves a %dx%d frame" % (args.scale, ow, oh))

    print("=" * 72)
    print("rawrec -> mp4")
    print("  in          %s" % src)
    print("  out         %s" % out)
    print("  frames      %d valid, %dx%d, %d bpp, %s"
          % (len(frames), W, H, meta["bpp"], meta.get("pixfmt", "?")))
    print("  rate        %.3f fps measured from t_mono" % fps_meas)
    adv = meta.get("advertised_fps")
    if adv and abs(adv - fps_meas) > 0.5:
        print("              header says advertised_fps %.1f -- IGNORED, it is "
              "what the core" % adv)
        print("              was asked for, not what it delivered")
    if args.fps:
        print("  rate        %.3f fps FORCED by --fps" % fps)
    print("  span        %.1f s recorded" % span)
    if args.scale != 1.0:
        print("  scale       %.2f -> %dx%d" % (args.scale, ow, oh))
    print("  overlay     %s" % ("off" if args.no_overlay else "rec_idx + time"))
    if args.telemetry:
        print("  telemetry   %s (CUBE-valid banner, ROI, LOS marker, cue)" % csv_path)
    print("  timebase    %s" % ("as-captured: frames back to back, video time "
                                "will NOT match real time" if args.as_captured
                                else "real: video time == recording time"))

    stretch = stretch_params(src, meta, frames)
    if stretch:
        print("  stretch     16-bit, global window %.0f-%.0f" % stretch)

    # ---- the output schedule ------------------------------------------------
    # A list of (source_index, t_rel, held) -- built up front so the frame count
    # is known before ffmpeg starts, and so a gap can be reported rather than
    # silently smoothed over.
    lo_t = t0 + args.start
    hi_t = t0 + args.end if args.end is not None else frames[-1][1]
    if hi_t <= lo_t:
        sys.exit("--start %.3f / --end %s selects nothing" % (args.start, args.end))

    sched, held_count = [], 0
    if args.as_captured:
        for i, (_, tm) in enumerate(frames):
            if lo_t <= tm <= hi_t:
                sched.append((i, tm - t0, False))
    else:
        n_out = int(round((hi_t - lo_t) * fps)) + 1
        j = 0
        for k in range(n_out):
            t = lo_t + k / fps
            while j + 1 < len(frames) and frames[j + 1][1] <= t:
                j += 1
            # A frame is "held" when the next real one is more than one output
            # period away: that is a genuine gap in the capture, not rounding.
            nxt = frames[j + 1][1] if j + 1 < len(frames) else frames[j][1]
            held = (t - frames[j][1]) > (1.5 / fps) and nxt > t
            held_count += held
            sched.append((j, t - t0, held))

    if not sched:
        sys.exit("nothing to write in the selected range")
    print("  writing     %d frames = %.1f s of video"
          % (len(sched), len(sched) / fps))
    if held_count:
        print("  GAPS        %d output frames are held repeats (%.2f s of the"
              % (held_count, held_count / fps))
        print("              recording is missing -- dropped at capture time)")

    cmd = [ff, "-hide_banner", "-loglevel", "error", "-y",
           "-f", "rawvideo", "-pix_fmt", "bgr24" if args.telemetry else "gray",
           "-s", "%dx%d" % (W, H), "-r", "%.6f" % fps, "-i", "-",
           "-an", "-c:v", "libx264", "-preset", args.preset, "-crf", str(args.crf),
           "-pix_fmt", "yuv420p", "-movflags", "+faststart"]
    if (ow, oh) != (W, H):
        cmd += ["-vf", "scale=%d:%d:flags=area" % (ow, oh)]
    cmd += [str(out)]

    print("-" * 72, flush=True)
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    t_start = time.monotonic()
    written = 0
    cache_i, cache_img = -1, None
    try:
        with open(src, "rb") as f:
            for (i, t_rel, held) in sched:
                if i != cache_i:
                    f.seek(frames[i][0])
                    buf = f.read(fb)
                    if len(buf) < fb:
                        break
                    cache_img = to8(buf, meta, stretch)
                    cache_i = i
                img = cache_img
                if args.telemetry:
                    # cvtColor always allocates a fresh array, so this is
                    # already safe to mutate -- unlike cache_img itself, which
                    # is reused across a held gap and must not be drawn on.
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    img = tv.annotate_from_row(img, per_frame[i], overlays=True)
                if not args.no_overlay:
                    if not args.telemetry:
                        img = img.copy()  # see above: cache_img must stay clean
                    img = draw_overlay(img, i, t_rel, frames[i][1], held,
                                       mag=1.0 / args.scale if args.scale else 1.0)
                proc.stdin.write(img.tobytes())
                written += 1
                if written % 250 == 0:
                    el = time.monotonic() - t_start
                    print("  %6d/%d  %.0f%%  %.1f fps encode  eta %.0fs"
                          % (written, len(sched), 100.0 * written / len(sched),
                             written / el, (len(sched) - written) / (written / el)),
                          flush=True)
    except (BrokenPipeError, KeyboardInterrupt) as e:
        print("  stopped: %s" % type(e).__name__)
    finally:
        try:
            proc.stdin.close()
        except OSError:
            pass
        rc = proc.wait()

    el = time.monotonic() - t_start
    print("-" * 72)
    if rc != 0:
        print("ffmpeg exited %d -- output may be unusable" % rc)
        return 1
    sz = out.stat().st_size if out.exists() else 0
    print("  wrote       %s" % out)
    print("  %d frames, %.1f s of video, %.1f MB (%.0f%% of the raw %.1f MB)"
          % (written, written / fps, sz / 1e6,
             100.0 * sz / max(1, len(sched) * fb), len(sched) * fb / 1e6))
    print("  took        %.0f s (%.1f fps encode)" % (el, written / el if el else 0))
    print("  THE .rawrec IS THE ARCHIVE -- keep it; this mp4 is lossy")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
