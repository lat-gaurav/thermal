#!/usr/bin/env python3
"""Step through a .rawrec thermal capture frame by frame.

    rawrec_viewer.py FILE.rawrec

KEYS
    right / n / .   next frame
    left / p / ,    previous frame
    space           play / pause
    [ / ]           slower / faster playback
    d               cycle detector (includes "none" to turn detection off)
    a               annotation on / off
    q / esc         quit

Hovering the mouse over the frame shows its pixel coordinate and raw value
in the strip above the image.

Detectors are loaded from the detector/ folder next to this repo: any .py file
there exposing a detect(frame) -> [(x, y, w, h), ...] function is picked up
automatically.
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

DETECTOR_DIR = pathlib.Path(__file__).resolve().parent.parent / "detector"

FILE_HDR = 4096            # bytes of file header before the first record
REC_MAGIC = 0xA5F00DEC     # per-record header magic, little-endian u32
WIN = "rawrec viewer"

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

    frame_idx = 0
    playing = False
    speed = 1.0
    last_tick = time.monotonic()
    raw = read_frame(frame_idx)
    disp = to_display(raw, bpp)
    boxes = run_detector(raw)

    mouse = {"x": None, "y": None}

    def on_mouse(event, x, y, flags, param):
        mouse["x"], mouse["y"] = x, y - HUD_H

    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN, on_mouse)

    while True:
        now = time.monotonic()
        if playing and now - last_tick >= base_interval / speed:
            last_tick = now
            if frame_idx < total - 1:
                frame_idx += 1
                raw = read_frame(frame_idx)
                disp = to_display(raw, bpp)
                boxes = run_detector(raw)
            else:
                playing = False

        header = np.zeros((HUD_H, w, 3), dtype=np.uint8)
        text = f"frame {frame_idx + 1}/{total}  {'PLAY' if playing else 'PAUSE'} ({speed:.2g}x)"
        mx, my = mouse["x"], mouse["y"]
        if mx is not None and 0 <= mx < w and 0 <= my < h:
            text += f"    x={mx} y={my}  val={int(raw[my, mx])}"
        text += f"    detector={detectors[det_idx][0]} [{'ON' if annotate else 'OFF'}]"
        cv2.putText(header, text, (10, HUD_H - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

        view = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)
        if annotate:
            for (x, y, bw, bh) in boxes:
                cv2.rectangle(view, (x, y), (x + bw, y + bh), BOX_COLOR, 1)
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
        elif (k in NEXT_KEYS or c in NEXT_KEYS) and frame_idx < total - 1:
            frame_idx += 1
            raw = read_frame(frame_idx)
            disp = to_display(raw, bpp)
            boxes = run_detector(raw)
        elif (k in PREV_KEYS or c in PREV_KEYS) and frame_idx > 0:
            frame_idx -= 1
            raw = read_frame(frame_idx)
            disp = to_display(raw, bpp)
            boxes = run_detector(raw)
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
