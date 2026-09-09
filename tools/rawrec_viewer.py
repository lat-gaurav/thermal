#!/usr/bin/env python3
"""Step through a .rawrec thermal capture frame by frame.

    rawrec_viewer.py FILE.rawrec

KEYS
    click           set/replace the LOS reference point at the current frame
    right / n / .   next frame
    left / p / ,    previous frame
    space           play / pause
    [ / ]           slower / faster playback
    d               cycle detector (includes "none" to turn detection off)
    a               annotation on / off
    q / esc         quit

Hovering the mouse over the frame shows its pixel coordinate and raw value
in the strip above the image. Drag the "frame" slider at the top of the
window to jump straight to a frame number.

Detectors are loaded from the detector/ folder next to this repo: any .py file
there exposing a detect(frame) -> [(x, y, w, h), ...] function is picked up
automatically.

If a matching los-*.csv is found (see experiment/los_static_track.py for the
matching/attitude math, reused here), clicking on the image marks a point
assumed static in the world; every other frame then shows where gimbal
attitude alone predicts that point should be, drawn as a small cross. This
is the same input filters/ folder detections get filtered against -- any
.py file there exposing a filter(boxes, context) -> boxes function runs on
the detector's output every frame, context["los_point"] being that
prediction (or None if no reference is set / no los data exists for this
file).
"""
import argparse
import importlib.util
import json
import os
import pathlib
import struct
import sys
import time

import cv2
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DETECTOR_DIR = REPO_ROOT / "detector"
FILTER_DIR = REPO_ROOT / "filters"

FILE_HDR = 4096            # bytes of file header before the first record
REC_MAGIC = 0xA5F00DEC     # per-record header magic, little-endian u32
WIN = "rawrec viewer"
TRACKBAR = "frame"

NEXT_KEYS = (63235, 0x270000, ord('n'), ord('.'))
PREV_KEYS = (63234, 0x250000, ord('p'), ord(','))
PLAY_KEYS = (ord(' '),)
SLOWER_KEYS = (ord('['),)
FASTER_KEYS = (ord(']'),)
DETECTOR_KEYS = (ord('d'),)
ANNOTATE_KEYS = (ord('a'),)
QUIT_KEYS = (27, ord('q'))

HUD_H = 30      # height in px of the info strip drawn above the frame
BOX_COLOR = (0, 200, 255)
SPEED_STEP = 1.5
MIN_SPEED = 1.0 / 16
MAX_SPEED = 16.0

LOS_REF_COLOR = (0, 255, 255)   # the clicked point, at its own frame
LOS_PRED_COLOR = (0, 0, 255)    # the predicted point, on every other frame
REANCHOR_EVERY = 500  # re-anchor the LOS reference to the nearest detection this
                    # often, so attitude/timing drift over a long dead-reckoned
                    # run gets periodically corrected against a real detection
                    # instead of compounding for the rest of the file
LOS_LATENCY_S = -0.070  # world-event -> attitude-lookup shift, applied on top of
                     # the los-*.csv's own qw..qz. Left at 0: the recording
                     # pipeline was found to already apply its own --latency-ms
                     # (200ms on the deployed rig) before writing the csv, so
                     # correcting again here would double-count it. See
                     # experiment/los_static_track.py's history for how that
                     # was found and confirmed.


def load_detectors():
    """Load every detect(frame) function found in the detector/ folder."""
    detectors = []
    if not DETECTOR_DIR.is_dir():
        return detectors
    for path in sorted(DETECTOR_DIR.glob("*.py")):
        if path.name.startswith("_"):
            continue
        spec = importlib.util.spec_from_file_location(path.stem, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        if hasattr(mod, "detect"):
            detectors.append((path.stem, mod.detect))
    return detectors


def load_filters():
    """Load every filter(boxes, context) function found in the filters/ folder."""
    filters = []
    if not FILTER_DIR.is_dir():
        return filters
    for path in sorted(FILTER_DIR.glob("*.py")):
        if path.name.startswith("_"):
            continue
        spec = importlib.util.spec_from_file_location(path.stem, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        if hasattr(mod, "filter"):
            filters.append((path.stem, mod.filter))
    return filters


def _load_los_track():
    """Reuse the attitude/reprojection math from experiment/los_static_track.py."""
    spec = importlib.util.spec_from_file_location(
        "los_static_track", REPO_ROOT / "experiment" / "los_static_track.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_header(path):
    """Parse the 4096-byte file header: 'RAWREC' magic + JSON descriptor at 0x80."""
    with open(path, "rb") as f:
        hdr = f.read(FILE_HDR)
    if len(hdr) < FILE_HDR or not hdr.startswith(b"RAWREC"):
        sys.exit(f"{path}: not a .rawrec (missing RAWREC magic)")
    meta = json.loads(hdr[0x80:].split(b"\x00")[0].decode())
    for key in ("width", "height", "bpp", "frame_bytes", "rec_len"):
        if key not in meta:
            sys.exit(f"{path}: header JSON has no '{key}'")
    return meta


def build_index(path, meta):
    """(offset, t_mono) of every real frame, skipping zero-filled tail padding."""
    fb, rl = meta["frame_bytes"], meta["rec_len"]
    rechdr = rl - fb
    total = (os.path.getsize(path) - FILE_HDR) // rl
    frames = []
    with open(path, "rb") as f:
        for i in range(total):
            off = FILE_HDR + i * rl
            f.seek(off)
            hb = f.read(32)
            if len(hb) < 32:
                break
            magic, = struct.unpack_from("<I", hb, 0)
            if magic == REC_MAGIC:
                t_mono, = struct.unpack_from("<d", hb, 12)
                frames.append((off + rechdr, t_mono))
    if not frames:
        sys.exit(f"{path}: no valid frames found")
    return frames


def measured_fps(frames):
    """Median frame rate from the recorded timestamps (advertised_fps can be wrong)."""
    if len(frames) < 2:
        return 25.0
    dts = np.diff(np.array([t for _, t in frames]))
    dts = dts[dts > 0]
    if dts.size == 0:
        return 25.0
    return float(1.0 / np.median(dts))


def to_display(frame, bpp):
    """8-bit view of a frame for the screen. 16-bit frames get a 1-99 percentile stretch."""
    if bpp == 1:
        return frame
    lo, hi = np.percentile(frame, (1, 99))
    if hi <= lo:
        hi = lo + 1
    scaled = np.clip((frame.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255)
    return scaled.astype(np.uint8)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="the .rawrec capture to open")
    args = ap.parse_args()

    meta = read_header(args.file)
    w, h, bpp = meta["width"], meta["height"], meta["bpp"]
    dtype = np.uint8 if bpp == 1 else np.dtype("<u2")
    frames = build_index(args.file, meta)
    offsets = [off for off, _ in frames]
    total = len(offsets)
    base_interval = 1.0 / measured_fps(frames)

    los_track = _load_los_track()
    focal = meta.get("focal")
    cx, cy = w / 2.0, h / 2.0
    los_path = los_track.find_los_csv(args.file)
    quat_rows = los_track.load_quaternions(los_path) if los_path else []
    los_ts = np.array([r[0] for r in quat_rows]) if quat_rows else None
    quats = ([los_track.interp_quat(t - LOS_LATENCY_S, quat_rows, los_ts) for _, t in frames]
              if (focal and quat_rows) else None)

    f = open(args.file, "rb")

    def read_frame(idx):
        f.seek(offsets[idx])
        buf = f.read(meta["frame_bytes"])
        return np.frombuffer(buf, dtype=dtype).reshape(h, w)

    detectors = load_detectors()
    detectors.append(("none", lambda raw_frame: []))
    det_idx = 0
    annotate = True

    def run_detector(raw_frame):
        if not annotate:
            return []
        return detectors[det_idx][1](raw_frame)

    filters = load_filters()

    frame_idx = 0
    playing = False
    speed = 1.0
    last_tick = time.monotonic()
    raw = read_frame(frame_idx)
    disp = to_display(raw, bpp)
    boxes = run_detector(raw)

    los_ref = None  # {"frame_idx": int, "uv": (x, y)}, or None until clicked

    def current_los_point():
        """Predicted image position of the clicked point on the current frame."""
        if los_ref is None or quats is None:
            return None
        if frame_idx == los_ref["frame_idx"]:
            return los_ref["uv"]
        pred = los_track.project(los_ref["uv"], quats[los_ref["frame_idx"]],
                                  quats[frame_idx], focal, cx, cy)
        if pred is None:
            return None
        u1, v1 = pred
        return (u1, v1) if (0 <= u1 < w and 0 <= v1 < h) else None

    def maybe_reanchor_los():
        """Every REANCHOR_EVERY frames, snap the LOS reference to whichever
        detection (from the RAW, unfiltered detector output) sits nearest the
        current prediction -- correcting drift instead of trusting one click
        for the whole file."""
        nonlocal los_ref
        if los_ref is None or quats is None:
            return
        if frame_idx - los_ref["frame_idx"] < REANCHOR_EVERY:
            return
        predicted = current_los_point()
        if predicted is None or not boxes:
            return
        px, py = predicted

        def center(box):
            x, y, bw, bh = box
            return (x + bw / 2.0, y + bh / 2.0)

        nearest = min(boxes, key=lambda b: (center(b)[0] - px) ** 2 + (center(b)[1] - py) ** 2)
        los_ref = {"frame_idx": frame_idx, "uv": center(nearest)}

    mouse = {"x": None, "y": None}

    def on_mouse(event, x, y, flags, param):
        nonlocal los_ref
        mouse["x"], mouse["y"] = x, y - HUD_H
        if (quats is not None and event == cv2.EVENT_LBUTTONDOWN
                and 0 <= mouse["x"] < w and 0 <= mouse["y"] < h):
            los_ref = {"frame_idx": frame_idx, "uv": (mouse["x"], mouse["y"])}

    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN, on_mouse)

    def goto_frame(idx):
        nonlocal frame_idx, raw, disp, boxes
        idx = max(0, min(total - 1, idx))
        if idx == frame_idx:
            return
        frame_idx = idx
        raw = read_frame(frame_idx)
        disp = to_display(raw, bpp)
        boxes = run_detector(raw)
        maybe_reanchor_los()
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

        header = np.zeros((HUD_H, w, 3), dtype=np.uint8)
        text = f"frame {frame_idx + 1}/{total}  {'PLAY' if playing else 'PAUSE'} ({speed:.2g}x)"
        mx, my = mouse["x"], mouse["y"]
        if mx is not None and 0 <= mx < w and 0 <= my < h:
            text += f"    x={mx} y={my}  val={int(raw[my, mx])}"
        text += f"    detector={detectors[det_idx][0]} [{'ON' if annotate else 'OFF'}]"
        if quats is None:
            text += "    los=n/a"
        else:
            text += "    los=SET" if los_ref else "    los=none (click the target)"
        cv2.putText(header, text, (10, HUD_H - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

        los_point = current_los_point()

        view = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)
        if annotate:
            draw_boxes = boxes
            for _, fn in filters:
                draw_boxes = fn(draw_boxes, {"los_point": los_point})
            for (x, y, bw, bh) in draw_boxes:
                cv2.rectangle(view, (x, y), (x + bw, y + bh), BOX_COLOR, 1)
        if los_ref is not None:
            if frame_idx == los_ref["frame_idx"]:
                cv2.drawMarker(view, tuple(int(v) for v in los_ref["uv"]), LOS_REF_COLOR,
                                cv2.MARKER_CROSS, 14, 2)
            elif los_point is not None:
                cv2.drawMarker(view, (int(los_point[0]), int(los_point[1])), LOS_PRED_COLOR,
                                cv2.MARKER_CROSS, 14, 2)
        canvas = np.vstack([header, view])
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
            speed = max(MIN_SPEED, speed / SPEED_STEP)
        elif k in FASTER_KEYS or c in FASTER_KEYS:
            speed = min(MAX_SPEED, speed * SPEED_STEP)
        elif k in NEXT_KEYS or c in NEXT_KEYS:
            goto_frame(frame_idx + 1)
        elif k in PREV_KEYS or c in PREV_KEYS:
            goto_frame(frame_idx - 1)
        elif k in DETECTOR_KEYS or c in DETECTOR_KEYS:
            det_idx = (det_idx + 1) % len(detectors)
            boxes = run_detector(raw)
        elif k in ANNOTATE_KEYS or c in ANNOTATE_KEYS:
            annotate = not annotate
            boxes = run_detector(raw)

    f.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
