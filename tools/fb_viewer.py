#!/usr/bin/env python3
"""Live preview on the HDMI screen physically attached to the rpi5.

The rpi5 boots to a text console (multi-user.target, no X/Wayland), so
rawrec_viewer.py's cv2.imshow window has no display server to open on and
web_viewer.py only helps if you have a second machine on the network. This
draws straight into the KMS framebuffer (/dev/fb0) instead, which needs no
compositor and works fine when the only thing you have is the monitor plugged
into the Pi and an ssh session to start it from.

    fb_viewer.py --live /dev/thermal0          # raw feed, full frame rate
    fb_viewer.py --live /dev/thermal0 --detect tophat_scr
    fb_viewer.py FILE.rawrec                   # loop a recording

Ctrl-C to quit; the console is repainted on the way out. There are no
interactive controls on purpose -- you start this over ssh with no keyboard
on the Pi itself, so everything is a command-line flag.

Reuses rawrec_viewer.py's header parsing / detector loading / display
stretch and web_viewer.py's LiveSource camera thread rather than repeating
any of it; this file is only the framebuffer blit those two lack.
"""
import argparse
import mmap
import os
import pathlib
import signal
import sys
import time

import cv2
import numpy as np

import rawrec_viewer as rv
import web_viewer as wv
import config

FB_SYS = "/sys/class/graphics"
HUD_COLOR = (0, 255, 0)


class Framebuffer:
    """mmap'd /dev/fb0. Handles the 16bpp RGB565 the vc4 driver gives us here
    as well as the 32bpp some other modes come up in, and honours `stride`
    (bytes per row can exceed width*bpp, in which case a straight reshape of
    the whole buffer would shear the image)."""

    def __init__(self, dev="/dev/fb0"):
        node = os.path.basename(dev)
        def attr(name):
            with open(f"{FB_SYS}/{node}/{name}") as f:
                return f.read().strip()
        self.w, self.h = (int(v) for v in attr("virtual_size").split(","))
        self.bpp = int(attr("bits_per_pixel"))
        self.stride = int(attr("stride"))
        if self.bpp not in (16, 32):
            sys.exit(f"{dev}: {self.bpp}bpp framebuffer not supported (need 16 or 32)")
        self.fd = os.open(dev, os.O_RDWR)
        self.map = mmap.mmap(self.fd, self.stride * self.h,
                             mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)

    def blit(self, bgr):
        """bgr must already be exactly (h, w, 3) uint8 for the whole screen."""
        if self.bpp == 16:
            b = bgr[:, :, 0].astype(np.uint16)
            g = bgr[:, :, 1].astype(np.uint16)
            r = bgr[:, :, 2].astype(np.uint16)
            row = (((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)).astype("<u2")
        else:
            row = np.dstack([bgr, np.full(bgr.shape[:2], 255, np.uint8)])
        raw = row.tobytes()
        rowbytes = self.w * (self.bpp // 8)
        if self.stride == rowbytes:
            self.map.seek(0)
            self.map.write(raw)
        else:  # padded rows: copy one scanline at a time
            for y in range(self.h):
                self.map.seek(y * self.stride)
                self.map.write(raw[y * rowbytes:(y + 1) * rowbytes])

    def clear(self):
        self.map.seek(0)
        self.map.write(b"\x00" * (self.stride * self.h))

    def close(self):
        self.map.close()
        os.close(self.fd)


def fit(view, fb_w, fb_h):
    """Scale to fit the screen preserving aspect, letterboxed on black.

    Nearest-neighbour on the way up: this is a 1280x1024 thermal sensor where
    a target can be a handful of pixels, and interpolation smears exactly the
    small bright blobs the detector exists to find."""
    scale = min(fb_w / view.shape[1], fb_h / view.shape[0])
    nw, nh = int(view.shape[1] * scale), int(view.shape[0] * scale)
    interp = cv2.INTER_NEAREST if scale >= 1 else cv2.INTER_AREA
    small = cv2.resize(view, (nw, nh), interpolation=interp)
    canvas = np.zeros((fb_h, fb_w, 3), np.uint8)
    y0, x0 = (fb_h - nh) // 2, (fb_w - nw) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = small
    return canvas


def pick_detector(name):
    """(label, fn) for --detect, or (None, None) when running raw."""
    if not name:
        return None, None
    detectors = dict(rv.load_detectors())
    if name not in detectors:
        sys.exit(f"unknown detector {name!r}; available: "
                 f"{', '.join(detectors) or '(none in detector/)'}")
    return name, detectors[name]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="a .rawrec capture to loop")
    ap.add_argument("--live", metavar="DEVICE",
                    help="read a V4L2 camera instead, e.g. /dev/thermal0")
    ap.add_argument("--width", type=int, default=config.CAMERA_WIDTH,
                    help="live camera frame width")
    ap.add_argument("--height", type=int, default=config.CAMERA_HEIGHT,
                    help="live camera frame height")
    ap.add_argument("--detect", metavar="NAME",
                    help="run this detector from detector/ and box its hits. Off by "
                         "default: tophat_scr costs ~0.2s/frame, which turns a 50fps "
                         "feed into a slideshow")
    ap.add_argument("--fbdev", default=config.FB_DEVICE, help="framebuffer device")
    ap.add_argument("--max-fps", type=float, default=config.FB_MAX_FPS,
                    help="cap the redraw rate; the panel cannot show more than it "
                         "refreshes and the blit is pure CPU")
    ap.add_argument("--no-hud", action="store_true", help="hide the text overlay")
    args = ap.parse_args()

    if bool(args.file) == bool(args.live):
        sys.exit("pass exactly one of FILE or --live")

    det_name, det_fn = pick_detector(args.detect)

    if args.live:
        src = wv.LiveSource(args.live, args.width, args.height)
        bpp, total, offsets, fh, meta = 1, None, None, None, None
        label = f"live {args.live}"
    else:
        path = pathlib.Path(args.file)
        meta = rv.read_header(path)
        frames = rv.build_index(path, meta)
        offsets = [off for off, _ in frames]
        total = len(offsets)
        bpp = meta["bpp"]
        fh = open(path, "rb")
        dtype = np.uint8 if bpp == 1 else np.uint16
        src = None
        label = f"{path.name} ({total} frames)"

    fb = Framebuffer(args.fbdev)
    print(f"{label} -> {args.fbdev} {fb.w}x{fb.h}@{fb.bpp}bpp"
          f"{', detector ' + det_name if det_name else ''}")
    print("Ctrl-C to quit")

    stop = False
    def on_signal(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    idx = 0
    shown = 0
    t_start = time.monotonic()
    period = 1.0 / args.max_fps if args.max_fps > 0 else 0.0
    fps_now = 0.0

    try:
        while not stop:
            t0 = time.monotonic()

            if src is not None:
                raw = src.get()
                if raw is None:
                    time.sleep(0.01)
                    continue
            else:
                fh.seek(offsets[idx])
                raw = np.frombuffer(fh.read(meta["frame_bytes"]),
                                    dtype=dtype).reshape(meta["height"], meta["width"])
                idx = (idx + 1) % total

            view = cv2.cvtColor(rv.to_display(raw, bpp), cv2.COLOR_GRAY2BGR)

            boxes = []
            if det_fn is not None:
                boxes = det_fn(raw)
                for (x, y, bw, bh) in boxes:
                    cv2.rectangle(view, (x, y), (x + bw, y + bh), rv.BOX_COLOR, 1)

            canvas = fit(view, fb.w, fb.h)

            if not args.no_hud:
                hud = f"{label}  {fps_now:4.1f} fps"
                if det_name:
                    hud += f"  {det_name}: {len(boxes)}"
                if total:
                    hud += f"  frame {idx or total}/{total}"
                cv2.putText(canvas, hud, (16, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, HUD_COLOR, 2, cv2.LINE_AA)

            fb.blit(canvas)

            shown += 1
            if shown % 10 == 0:
                now = time.monotonic()
                fps_now = 10.0 / (now - t_start)
                t_start = now

            lag = period - (time.monotonic() - t0)
            if lag > 0:
                time.sleep(lag)
    finally:
        # Stop and drain the grab thread before anything else: it is parked in
        # a blocking cap.read(), and letting the interpreter tear down around
        # it aborts the process ("FATAL: exception not rethrown").
        if src is not None:
            src.running = False
            src.thread.join(timeout=2.0)
            src.cap.release()
        fb.clear()
        fb.close()
        if fh is not None:
            fh.close()
        # The console does not know we painted over it, so ask it to redraw.
        # Only root owns /dev/tty1, and over ssh we are not it -- a plain
        # cleared screen is a fine outcome, so this is best-effort.
        if os.access("/dev/tty1", os.W_OK):
            os.system("setterm --reset > /dev/tty1 2>/dev/null")
        print("\nstopped")


if __name__ == "__main__":
    main()
