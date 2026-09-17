#!/usr/bin/env python3
"""Replay a .rawrec next to the telemetry the pipeline itself logged for it.

    telemetry_viewer.py FILE.rawrec [--csv LOS.csv] [--scale 0.7]

The flight pipeline writes a los-*.csv carrying, per frame, everything it
decided at the time: where it thought the target was (los_x/los_y), whether
it was tracking or coasting, its gate radius and what fell inside it, the
detector's box counts before and after filtering, the crop it searched, the
attitude it used, the bearing it sent, and the Cube's own cue. This viewer
plays the frames back with all of that drawn and tabulated beside them, so a
flight can be inspected after the fact without guessing.

Nothing is re-run here -- no detector, no filters. Every number shown is
what was actually recorded, which is the point (and is why playback is
real-time rather than detector-bound).

The one exception is OUR OWN LOS reprojection, drawn for comparison: press
'i' to anchor it to whatever the pipeline logged as the target on the
current frame, then watch how a pure attitude-driven reprojection of that
point diverges from what the pipeline went on to log. '[' / ']' retune the
latency that reprojection uses, live. The panel reports the pixel gap
between the two, which is the number that says whether our LOS math and the
flight's agree.

A second, unrelated way to track something: press 's' then drag a box
around anything in the current frame, and a single-object tracker (see
sot/, selected by config.SOT_ALGO) follows that patch of pixels forward by
appearance alone -- no detector, no attitude, independent of everything
else this viewer draws. Useful for a target the logged pipeline missed, or
as a second opinion on where the LOS reprojection says the target went.

KEYS
    click           anchor our LOS reprojection at the clicked pixel
    i               anchor it to the pipeline's own logged LOS this frame
    [ / ]           our reprojection's latency: -/+ one step
    s               arm single-object tracker: drag a box to start it
    x               clear the single-object tracker
    right / n / .   next frame        left / p / ,   previous frame
    space           play / pause      [ / ] on playback is taken by latency,
    - / =           slower / faster   so speed uses - and = here
    a               overlays on / off
    q / esc         quit
"""
import argparse
import csv
import pathlib
import sys
import time

import cv2
import numpy as np

import rawrec_viewer as rv  # also bootstraps repo root onto sys.path
import config

WIN = "telemetry viewer"
TRACKBAR = "frame"

NEXT_KEYS = (63235, 0x270000, ord('n'), ord('.'))
PREV_KEYS = (63234, 0x250000, ord('p'), ord(','))
PLAY_KEYS = (ord(' '),)
SLOWER_KEYS = (ord('-'),)
FASTER_KEYS = (ord('='),)
LAT_DOWN_KEYS = (ord('['),)
LAT_UP_KEYS = (ord(']'),)
ANCHOR_KEYS = (ord('i'),)
OVERLAY_KEYS = (ord('a'),)
SOT_SELECT_KEYS = (ord('s'),)
SOT_CLEAR_KEYS = (ord('x'),)
QUIT_KEYS = (27, ord('q'))

# Columns pulled out of the csv, grouped the way the panel shows them. Any
# column missing from a given file's schema is simply skipped, so this keeps
# working against both the older and newer los-*.csv layouts.
GROUPS = [
    ("tracker", ["los_status", "los_x", "los_y", "gate_px", "n_in_gate",
                  "det_score", "match_dist_px", "alpha", "misses", "omega_deg_s",
                  "frames_since_update", "coast"]),
    ("detector", ["roi", "n_boxes", "n_kept", "det_ms", "loop_ms", "skipped",
                   "drops", "raw_dropped", "metric"]),
    ("attitude", ["qw", "qx", "qy", "qz", "att_age_ms", "att_hz"]),
    ("bearing", ["az_deg", "el_deg", "body_az_deg", "body_el_deg",
                  "los_n", "los_e", "los_d", "det_valid", "valid_hold",
                  "seq_sent", "az_cam_deg", "el_cam_deg", "ego_deg"]),
    ("cue", ["cue_valid", "cue_az_deg", "cue_el_deg", "cue_range_m",
              "cue_age_ms", "cue_u", "cue_v", "cue_resid_deg", "cue_bad_frames",
              "acq_via"]),
    ("state", ["algo", "arm_reason", "rc_arm_us", "recording", "rec_reason",
                "rc_rec_us", "episode", "rec_frames", "rec_idx", "cube_armed",
                "hb_age_ms", "grabbed", "mode"]),
]

STATUS_COLORS = {
    "tracking": config.VIEWER_LOS_TRACK_COLOR,
    "coasting": config.VIEWER_LOS_COAST_COLOR,
    "dropped": config.TELEMETRY_LOGGED_DROP_COLOR,
}


def load_telemetry(csv_path):
    """All rows with a usable t_mono, sorted, plus the column names present."""
    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        for row in reader:
            try:
                row["_t"] = float(row["t_mono"])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append(row)
    rows.sort(key=lambda r: r["_t"])
    return rows, fields


def join_by_t_mono(frame_times, rows):
    """One csv row per frame, chosen by nearest t_mono.

    Not by rec_idx: on these logs rec_idx lands up to 80ms away from the
    frame it claims, while t_mono matches exactly -- the same reason the
    rpi5's own rawrec_quats.py joins on time rather than on an index.
    """
    if not rows:
        return [None] * len(frame_times)
    ts = np.array([r["_t"] for r in rows])
    out = []
    for t in frame_times:
        j = int(np.searchsorted(ts, t))
        cands = [k for k in (j - 1, j) if 0 <= k < len(rows)]
        out.append(rows[min(cands, key=lambda k: abs(ts[k] - t))])
    return out


def fnum(row, key):
    """Float from a csv cell, or None if absent/blank/non-numeric."""
    if row is None:
        return None
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="the .rawrec capture to replay")
    ap.add_argument("--csv", help="override: path to the matching los-*.csv")
    ap.add_argument("--scale", type=float, default=1.0,
                     help="shrink the displayed frame (the panel is added beside it)")
    args = ap.parse_args()

    meta = rv.read_header(args.file)
    w, h, bpp = meta["width"], meta["height"], meta["bpp"]
    dtype = np.uint8 if bpp == 1 else np.dtype("<u2")
    frames = rv.build_index(args.file, meta)
    offsets = [off for off, _ in frames]
    frame_times = [t for _, t in frames]
    total = len(offsets)
    base_interval = 1.0 / rv.measured_fps(frames)

    los_track = rv._load_los_track()
    csv_path = pathlib.Path(args.csv) if args.csv else los_track.find_los_csv(args.file)
    if csv_path is None or not pathlib.Path(csv_path).is_file():
        sys.exit(f"{args.file}: no matching los-*.csv found (pass --csv to point at one)")
    rows, fields = load_telemetry(csv_path)
    if not rows:
        sys.exit(f"{csv_path}: no rows with a usable t_mono")
    per_frame = join_by_t_mono(frame_times, rows)
    present = [(title, [k for k in keys if k in fields]) for title, keys in GROUPS]
    present = [(title, keys) for title, keys in present if keys]

    # Our own reprojection needs attitude on the same clock. The csv's
    # qw..qz columns are the same in both schemas, so this works either way.
    focal = meta.get("focal")
    cx, cy = w / 2.0, h / 2.0
    quat_rows = los_track.load_quaternions(csv_path)
    quat_ts = np.array([r[0] for r in quat_rows]) if quat_rows else None
    latency = config.LOS_LATENCY_S
    quats = None

    def rebuild_quats():
        nonlocal quats
        if not (focal and quat_rows):
            quats = None
            return
        quats = [los_track.interp_quat(t - latency, quat_rows, quat_ts) for t in frame_times]

    rebuild_quats()

    f = open(args.file, "rb")

    def read_frame(idx):
        f.seek(offsets[idx])
        buf = f.read(meta["frame_bytes"])
        return np.frombuffer(buf, dtype=dtype).reshape(h, w)

    frame_idx = 0
    playing = False
    speed = 1.0
    overlays = True
    last_tick = time.monotonic()
    raw = read_frame(frame_idx)
    disp = rv.to_display(raw, bpp)
    ours = None  # {"frame_idx": int, "uv": (x, y)}

    def our_point(idx):
        if ours is None or quats is None:
            return None
        if idx == ours["frame_idx"]:
            return ours["uv"]
        return los_track.project(ours["uv"], quats[ours["frame_idx"]], quats[idx],
                                  focal, cx, cy)

    # ---- single-object tracker: independent of the LOS/detector machinery
    # above, this just follows a hand-picked patch of pixels by appearance.
    sot_name, sot_create = rv.pick_sot(config.SOT_ALGO, rv.load_sots())
    sot = {
        "tracker": None,   # the stateful tracker object once (re)started
        "init_idx": None,  # frame it was last (re)initialised on
        "last_idx": None,  # furthest frame index it has been advanced to
        "boxes": {},        # frame_idx -> (ok, (x, y, w, h)), one per tracked frame
        "armed": False,     # True after 's', before the drag that starts it
        "select": None,     # {"x0","y0","x1","y1"} while a box is being dragged
    }

    def frame_bgr(idx):
        raw_i = raw if idx == frame_idx else read_frame(idx)
        return cv2.cvtColor(rv.to_display(raw_i, bpp), cv2.COLOR_GRAY2BGR)

    def start_sot(idx, box):
        if sot_create is None:
            return
        tr = sot_create()
        tr.init(frame_bgr(idx), box)
        sot["tracker"] = tr
        sot["init_idx"] = idx
        sot["last_idx"] = idx
        sot["boxes"] = {idx: (True, box)}

    def sot_box_at(idx):
        """(ok, (x,y,w,h)) at frame idx, or None if untracked there.

        A correlation tracker cannot be asked to "jump": its internal
        appearance model is only valid one frame ahead of wherever it last
        ran. So reaching a frame past the cache means stepping it forward
        through every intervening frame first, not just evaluating idx
        directly -- slower for a big slider jump, but it is the only way the
        answer at idx means anything.
        """
        if sot["tracker"] is None or idx < sot["init_idx"]:
            return None
        if idx not in sot["boxes"]:
            for j in range(sot["last_idx"] + 1, idx + 1):
                sot["boxes"][j] = sot["tracker"].update(frame_bgr(j))
                sot["last_idx"] = j
        return sot["boxes"].get(idx)

    mouse = {"x": None, "y": None}

    def on_mouse(event, x, y, flags, param):
        nonlocal ours
        ix, iy = int(x / args.scale), int(y / args.scale)
        mouse["x"], mouse["y"] = ix, iy
        if sot["armed"]:
            if event == cv2.EVENT_LBUTTONDOWN and 0 <= ix < w and 0 <= iy < h:
                sot["select"] = {"x0": ix, "y0": iy, "x1": ix, "y1": iy}
            elif event == cv2.EVENT_MOUSEMOVE and sot["select"] is not None:
                sot["select"]["x1"], sot["select"]["y1"] = ix, iy
            elif event == cv2.EVENT_LBUTTONUP and sot["select"] is not None:
                s = sot["select"]
                x0, x1 = sorted((s["x0"], s["x1"]))
                y0, y1 = sorted((s["y0"], s["y1"]))
                sot["select"] = None
                sot["armed"] = False
                bw, bh = x1 - x0, y1 - y0
                if (bw >= config.TELEMETRY_SOT_MIN_BOX_PX
                        and bh >= config.TELEMETRY_SOT_MIN_BOX_PX):
                    start_sot(frame_idx, (float(x0), float(y0), float(bw), float(bh)))
            return
        if event == cv2.EVENT_LBUTTONDOWN and 0 <= ix < w and 0 <= iy < h:
            ours = {"frame_idx": frame_idx, "uv": (float(ix), float(iy))}

    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN, on_mouse)

    def goto_frame(idx):
        nonlocal frame_idx, raw, disp
        idx = max(0, min(total - 1, idx))
        if idx == frame_idx:
            return
        frame_idx = idx
        raw = read_frame(frame_idx)
        disp = rv.to_display(raw, bpp)
        if total > 1:
            cv2.setTrackbarPos(TRACKBAR, WIN, frame_idx)

    if total > 1:
        cv2.createTrackbar(TRACKBAR, WIN, 0, total - 1, goto_frame)

    while True:
        now = time.monotonic()
        if playing and now - last_tick >= base_interval / speed:
            last_tick = now
            if frame_idx < total - 1:
                goto_frame(frame_idx + 1)
            else:
                playing = False

        row = per_frame[frame_idx]
        view = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)

        logged = (fnum(row, "los_x"), fnum(row, "los_y"))
        status = (row or {}).get("los_status", "") or ""
        pred = our_point(frame_idx)

        if overlays:
            # the crop the pipeline searched, centred on its own LOS -- only
            # the size is logged, not the origin, so the centre is inferred
            roi = (row or {}).get("roi", "")
            if roi and roi != "FULL" and "x" in roi and None not in logged:
                try:
                    rw, rh = (int(v) for v in roi.split("x"))
                    x0, y0 = int(logged[0] - rw / 2), int(logged[1] - rh / 2)
                    cv2.rectangle(view, (x0, y0), (x0 + rw, y0 + rh),
                                   config.TELEMETRY_ROI_COLOR, 1)
                except ValueError:
                    pass

            if None not in logged:
                colour = STATUS_COLORS.get(status, config.TELEMETRY_LOGGED_NONE_COLOR)
                cv2.drawMarker(view, (int(logged[0]), int(logged[1])), colour,
                                cv2.MARKER_SQUARE, 18, 2)

            cue = (fnum(row, "cue_u"), fnum(row, "cue_v"))
            if fnum(row, "cue_valid") and None not in cue:
                cv2.drawMarker(view, (int(cue[0]), int(cue[1])),
                                config.TELEMETRY_CUE_COLOR, cv2.MARKER_TILTED_CROSS, 18, 2)

            if pred is not None and 0 <= pred[0] < w and 0 <= pred[1] < h:
                cv2.drawMarker(view, (int(pred[0]), int(pred[1])),
                                config.TELEMETRY_OURS_COLOR, cv2.MARKER_CROSS, 16, 2)

            sot_here = sot_box_at(frame_idx)
            if sot_here is not None:
                sot_ok, (sbx, sby, sbw, sbh) = sot_here
                colour = (config.TELEMETRY_SOT_COLOR if sot_ok
                          else config.TELEMETRY_SOT_LOST_COLOR)
                cv2.rectangle(view, (int(sbx), int(sby)),
                               (int(sbx + sbw), int(sby + sbh)), colour, 2)

        if sot["select"] is not None:  # the box being dragged out, shown even
            s = sot["select"]          # with overlays off -- it is live input,
            x0, x1 = sorted((s["x0"], s["x1"]))  # not a data overlay
            y0, y1 = sorted((s["y0"], s["y1"]))
            cv2.rectangle(view, (x0, y0), (x1, y1),
                           config.TELEMETRY_SOT_SELECT_COLOR, 1)

        if args.scale != 1.0:
            view = cv2.resize(view, None, fx=args.scale, fy=args.scale,
                               interpolation=cv2.INTER_AREA)

        # ---- panel ----------------------------------------------------------
        lines = []  # (text, colour, indent)

        def line(text, colour=(0, 255, 0), indent=0):
            lines.append((text, colour, indent))

        line(f"frame {frame_idx + 1}/{total}")
        line(f"t={frame_times[frame_idx]:.3f}")
        line(f"{'PLAY' if playing else 'PAUSE'} {speed:.2g}x "
             f"{'ovl' if overlays else 'no-ovl'}")
        if sot["armed"]:
            line("SOT: drag a box on the frame to start tracking", (0, 255, 255))
        mx, my = mouse["x"], mouse["y"]
        if mx is not None and 0 <= mx < w and 0 <= my < h:
            line(f"cursor {mx},{my} v={int(raw[my, mx])}", (160, 160, 160))
        else:
            line("")

        for title, keys in present:
            line("")
            line(f"-- {title} " + "-" * max(0, 22 - len(title)), (0, 180, 255))
            for k in keys:
                v = (row or {}).get(k, "")
                line(f"{k:<15s}{v if v != '' else '-'}", indent=4)

        line("")
        line("-- ours ---------------", (0, 180, 255))
        if quats is None:
            line("unavailable (no att)", (160, 160, 160), indent=4)
        else:
            line(f"{'latency':<15s}{latency * 1000:+.0f}ms", indent=4)
            if ours is None:
                line("click / 'i' to anchor", (160, 160, 160), indent=4)
            elif pred is None:
                line(f"{'pred':<15s}behind cam", (160, 160, 160), indent=4)
            else:
                line(f"{'pred x':<15s}{pred[0]:.1f}", indent=4)
                line(f"{'pred y':<15s}{pred[1]:.1f}", indent=4)
                line(f"{'anchor frame':<15s}{ours['frame_idx'] + 1}", indent=4)
                if None not in logged:
                    d = ((pred[0] - logged[0]) ** 2 + (pred[1] - logged[1]) ** 2) ** 0.5
                    line(f"{'gap vs logged':<15s}{d:.1f}px",
                         (0, 255, 0) if d < 40 else (0, 165, 255), indent=4)

        line("")
        line("-- sot -----------------", (0, 180, 255))
        if sot_create is None:
            line("unavailable (no sot/)", (160, 160, 160), indent=4)
        elif sot["tracker"] is None:
            line("'s' then drag to start", (160, 160, 160), indent=4)
        else:
            here = sot_box_at(frame_idx)
            line(f"{'algo':<15s}{sot_name}", indent=4)
            line(f"{'init frame':<15s}{sot['init_idx'] + 1}", indent=4)
            if here is None:
                line("before init frame", (160, 160, 160), indent=4)
            else:
                ok, (bx, by, bw, bh) = here
                line(f"{'status':<15s}{'tracking' if ok else 'LOST'}",
                     (0, 255, 0) if ok else (0, 0, 255), indent=4)
                line(f"{'box':<15s}{bx:.0f},{by:.0f} {bw:.0f}x{bh:.0f}", indent=4)

        # Columnise so nothing is clipped: enough columns that the tallest
        # one fits beside the frame, then grow the canvas if even that fails.
        lh = config.TELEMETRY_LINE_H
        usable = max(view.shape[0] - lh, lh)
        ncols = max(1, -(-len(lines) * lh // usable))          # ceil division
        per_col = -(-len(lines) // ncols)                       # ceil division
        panel_h = max(view.shape[0], per_col * lh + lh)
        panel = np.zeros((panel_h, ncols * config.TELEMETRY_COL_W, 3), dtype=np.uint8)
        for i, (text, colour, indent) in enumerate(lines):
            col, rowi = divmod(i, per_col)
            cv2.putText(panel, text,
                        (8 + indent + col * config.TELEMETRY_COL_W, (rowi + 1) * lh),
                        cv2.FONT_HERSHEY_SIMPLEX, config.TELEMETRY_FONT_SCALE,
                        colour, 1, cv2.LINE_AA)

        if panel.shape[0] != view.shape[0]:  # pad the shorter side, never crop
            pad = np.zeros((panel.shape[0] - view.shape[0], view.shape[1], 3), dtype=np.uint8)
            view = np.vstack([view, pad])
        canvas = np.hstack([view, panel])
        cv2.imshow(WIN, canvas)

        k = cv2.waitKeyEx(30)
        if k == -1:
            continue
        c = k & 0xFF

        if k in QUIT_KEYS or c in QUIT_KEYS:
            break
        elif k in PLAY_KEYS or c in PLAY_KEYS:
            playing = not playing
            last_tick = time.monotonic()
        elif k in SLOWER_KEYS or c in SLOWER_KEYS:
            speed = max(config.VIEWER_MIN_SPEED, speed / config.VIEWER_SPEED_STEP)
        elif k in FASTER_KEYS or c in FASTER_KEYS:
            speed = min(config.VIEWER_MAX_SPEED, speed * config.VIEWER_SPEED_STEP)
        elif k in NEXT_KEYS or c in NEXT_KEYS:
            goto_frame(frame_idx + 1)
        elif k in PREV_KEYS or c in PREV_KEYS:
            goto_frame(frame_idx - 1)
        elif k in LAT_DOWN_KEYS or c in LAT_DOWN_KEYS:
            latency = max(config.LOS_MIN_LATENCY_S, latency - config.LOS_LATENCY_STEP_S)
            rebuild_quats()
        elif k in LAT_UP_KEYS or c in LAT_UP_KEYS:
            latency = min(config.LOS_MAX_LATENCY_S, latency + config.LOS_LATENCY_STEP_S)
            rebuild_quats()
        elif k in ANCHOR_KEYS or c in ANCHOR_KEYS:
            if None not in logged:
                ours = {"frame_idx": frame_idx, "uv": (logged[0], logged[1])}
        elif k in OVERLAY_KEYS or c in OVERLAY_KEYS:
            overlays = not overlays
        elif k in SOT_SELECT_KEYS or c in SOT_SELECT_KEYS:
            sot["armed"] = True
            sot["select"] = None
        elif k in SOT_CLEAR_KEYS or c in SOT_CLEAR_KEYS:
            sot["tracker"] = None
            sot["init_idx"] = None
            sot["last_idx"] = None
            sot["boxes"] = {}
            sot["armed"] = False
            sot["select"] = None

    f.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
