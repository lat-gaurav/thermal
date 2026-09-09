#!/usr/bin/env python3
"""Step through a .rawrec thermal capture frame by frame, with no encoding loss.

Reads frames straight out of the .rawrec. Nothing is decoded, re-quantised or
re-compressed on the way to the screen: the pixels you inspect are the bytes the
sensor wrote. The mp4 produced by the conversion pipeline is H.264 and therefore
lossy -- use this when you need the real values, not a preview.

    rawrec_view.py FILE.rawrec              open the viewer
    rawrec_view.py FILE.rawrec -f 4200      open at a specific frame
    rawrec_view.py FILE.rawrec --info       print header + index summary, no GUI
    rawrec_view.py FILE.rawrec --probe 900  dump one frame's stats, no GUI

KEYS

    right / n / .      next frame            left / p / ,   previous frame
    shift-right / N    +10 frames            shift-left / P  -10 frames
    home / end         first / last frame    g              jump to frame (asks)
    space              play / pause          [ / ]          slower / faster
    z                  toggle fit <-> 1:1    + / -          zoom in / out
    drag               pan when zoomed in    c              cycle colormap
    a                  raw image (no marks)  i              frame info to stdout
    h                  toggle the HUD text
    s                  save frame as PNG     q / esc        quit
    v                  toggle profile panel  l              lock / free the probe
    d                  detector on / off     e / r          min_contrast -/+ 5
    t / y              max_crowd -/+ 5       m              detection overlay
    k                  echo current flags    K              reset to defaults

PROFILE PANEL

Beside the image, four plots cut through whatever pixel the mouse is over:
the intensity along that row and its first derivative dI/dx, then the intensity
down that column and its derivative dI/dy. The header prints I, both derivatives,
and the gradient magnitude and direction at the probe.

Derivative plots use a fixed +/-25 DN/px axis so the same slope looks the same
from frame to frame; autoscaling would rescale the axis under a growing peak and
hide the very change you are watching for. Excursions past the scale are clamped
inside the plot, ticked red on the border they left through, and the true extreme
is printed beside the title -- so a clipped peak is loud, not lost. --grad-scale
changes the range.

The cuts are taken from the full-resolution source row and column, never from
the zoomed or colormapped display, so the derivative is the sensor's, not the
scaler's. Plot columns are drawn as a min/max envelope rather than by sampling:
a one-pixel gradient spike -- the edge you opened a profile to find -- cannot
fall between bins and vanish.

'l' freezes the probe where it is, so you can step frames and watch one edge
evolve without holding the mouse still.

SMALL-TARGET DETECTOR

Tuned for one thing: an object a few pixels across, sitting on its own. Cloud is
the clutter and cloud is extended, which is what the detector exploits.

  1. White top-hat -- the frame minus its greyscale opening. The opening erases
     anything narrower than TH_SIZE, so subtracting it removes cloud, horizon
     and every smooth gradient outright, however bright, and keeps small objects.
  2. Signal-to-clutter -- top-hat response over the mean |response| nearby. On
     smooth sky the denominator is a fraction of a DN and an object scores
     enormously; on a busy cloud edge the same response scores little. This is
     the isolation test, and it is what does the real work.
  3. Symmetry -- the profile must climb and then fall by comparable amounts on
     both axes, which rejects step edges.

Everything here operates on INTENSITY; no derivative is involved, and every
detection is a local maximum of the raw image. The dI/dx and dI/dy plots remain
purely for looking at. Survivors are ranked by SCR and merged, so one object
gives one detection, ringed in magenta on the image and ticked on the two
intensity profiles. Against four hand-checked frames the defaults give
exactly one detection, on the target, in every one:

    frame 596 -> (606,620)   frame 753 -> (730,585)
    frame 732 -> (639,485)   frame 734 -> (646,491)

Across 60 arbitrary frames it averaged 0.85 detections, never more than 2. Costs
~47 ms/frame, cached per frame and parameter set.

MIN_SCR is the parameter to reach for. It ranked the true target first in every
ground-truth frame, so raising it trims clutter from the bottom while the target
stays put -- unlike a plain contrast threshold, which loses the target first.

SEEING THE RAW IMAGE

A green cross marks the probed pixel on the image itself. It is drawn by
mapping the reported IMAGE pixel back to the screen, so if it does not sit under
your pointer the coordinate readout is wrong and you can see exactly by how
much. The HUD prints both spaces -- win(x,y) -> px(x,y) -- to make that
diagnosable. If the two are related by a constant factor, the window is being
scaled by the OS: pass --mouse-scale (2.0 on a HiDPI screen) to correct it.

'a' strips every mark from the image -- HUD text and peak overlay both -- and
restores them again. Zoom, pan and the panel are deliberately left alone, so the
picture does not move a pixel between the two states: flick 'a' to check what a
marker is sitting on top of, which is the whole reason to want the raw view.
'v' additionally hides the side panel if you want the image full-width, and 'h'
drops just the HUD text while keeping the peak marks.

Saving is unaffected either way -- 's' always writes the untouched source frame,
never the annotated display.

LIVE TUNING

Four sliders along the top of the window drive the detector -- tophat, min_th,
min_scr and max_width -- and the image overlay and header counts follow as you
drag. e/r nudge min_scr, t/y nudge min_th; sliders and keys stay in sync.

Park the probe on a target ('l' locks it) and the panel tells you which test is
failing -- "fails scr (not isolated)" is the usual answer on cloud.

Every change echoes the whole parameter set as a ready-made command line, so
whatever you tuned to is the last line in your terminal:

    tuning: --win 33 --min-prom 5 --max-width 8 --rel-height 0.5

'k' reprints it on demand and 'K' restores the defaults. Watch the "row / col /
BOTH" counts in the overlay HUD while you drag: BOTH is the number that matters,
and it should fall to a handful before the thresholds are worth trusting.

Dragging recomputes the full-frame overlay (~35 ms), so the window updates at
roughly 15 fps while a slider is moving. Turn the overlay off with 'm' if you
only care about the profiles.

'm' cycles the overlay on the image itself: all flagged pixels, then
coincidences only, then off. Single-axis hits are bare dots -- olive for a row
peak, blue for a column peak -- and a pixel flagged on both axes gets a magenta
ring. The distinction earns its keep: on a typical frame here the row and column
passes flag ~670 and ~780 pixels, and only ~90 of them coincide, so requiring
both discards about 87% of candidates without a single extra parameter.

Detection is per-axis and per-frame only. Nothing here tracks a target across
frames.

WHY THE PIXELS ARE TRUSTWORTHY

The frame is memory-mapped as its native dtype and never modified. Everything
that could alter a value -- zoom, colormap, the overlay -- happens on a separate
BGR copy made for the window. Two consequences worth knowing:

  * The pixel readout under the cursor reports the value from the untouched
    source array, so it is the true sensor reading regardless of zoom or palette.
  * 's' writes the source array to PNG. PNG is lossless, so for 8-bit captures
    the file is bit-exact; for 16-bit captures it is a 16-bit PNG, also exact.

At 1:1 and above the display scaler is nearest-neighbour, so one stored pixel is
one block on screen and nothing is invented between them. Zoomed out to fit, it
switches to area-averaging: plain subsampling would alias, and on this footage
aliasing can drop a small target out of the picture entirely, which matters more
for review than the purity of a downscaled preview. The HUD names the mode.

FORMAT. See rawrec_format() below -- the file carries its own JSON descriptor,
so geometry is read from the capture rather than assumed.
"""

import argparse
import json
import os
import struct
import sys
import warnings

import cv2
import numpy as np

try:    # the detector lives in thermal_detect; the viewer works without it
    from thermal_detect import (detect, features, MERGE,
                                MIN_CONTRAST, MIN_MOAT, MAX_CROWD)
    HAVE_DET = True
except ImportError:                    # pragma: no cover
    HAVE_DET = False
    MIN_CONTRAST, MIN_MOAT, MAX_CROWD, MERGE = 50.0, 2.0, 20, 6

FILE_HDR = 4096            # bytes of file header before the first record
REC_MAGIC = 0xA5F00DEC     # per-record header magic, little-endian u32

GRAD_SPAN = 25.0           # derivative plots use a fixed +/-25 DN/px axis

# Detection is not implemented here any more -- it is thermal_detect.detect, so
# the rings you see are exactly what `thermal_detect.py --run` writes. Keeping a
# second copy in the viewer meant the two silently disagreed about what a
# detection was.

COLORMAPS = [
    ("gray", None),
    ("inferno", cv2.COLORMAP_INFERNO),
    ("turbo", cv2.COLORMAP_TURBO),
    ("hot", cv2.COLORMAP_HOT),
]


def rawrec_format(path):
    """Return the capture's JSON descriptor.

    The file header is 4096 bytes: ASCII 'RAWREC' at 0, then a NUL-terminated
    JSON blob at 0x80 carrying width/height/bpp/pixfmt/frame_bytes/rec_len.
    Reading it instead of hardcoding geometry is what lets this work on captures
    whose .meta sidecar never made it off the recorder.
    """
    with open(path, "rb") as f:
        hdr = f.read(FILE_HDR)
    if len(hdr) < FILE_HDR or not hdr.startswith(b"RAWREC"):
        raise SystemExit(f"{path}: not a .rawrec (missing RAWREC magic)")
    try:
        meta = json.loads(hdr[0x80:].split(b"\x00")[0].decode())
    except (ValueError, UnicodeDecodeError) as e:
        raise SystemExit(f"{path}: header JSON is unreadable: {e}")
    for key in ("width", "height", "bpp", "frame_bytes", "rec_len"):
        if key not in meta:
            raise SystemExit(f"{path}: header JSON has no '{key}'")
    return meta


def build_index(path, meta, quiet=False):
    """Scan record headers and return the list of real frames.

    Records are fixed length, so an offset is arithmetic rather than a walk. The
    scan exists to read each record's metadata and, more importantly, to drop
    zero-filled tail padding: a recorder that is killed mid-flight leaves records
    whose magic is 0, and those are not frames.
    """
    w, h, bpp = meta["width"], meta["height"], meta["bpp"]
    fb, rl = meta["frame_bytes"], meta["rec_len"]
    if w * h * bpp != fb:
        raise SystemExit(
            f"{path}: {w}x{h}x{bpp}B = {w * h * bpp} but header says "
            f"frame_bytes={fb}; unsupported pixel layout "
            f"(pixfmt={meta.get('pixfmt')!r})")
    if bpp not in (1, 2):
        raise SystemExit(f"{path}: bpp={bpp} unsupported (expected 1 or 2)")

    rechdr = rl - fb
    if rechdr < 32:
        raise SystemExit(f"{path}: rec_len {rl} leaves no room for a record header")

    total = (os.path.getsize(path) - FILE_HDR) // rl
    frames, padding = [], 0
    with open(path, "rb") as f:
        for i in range(total):
            off = FILE_HDR + i * rl
            f.seek(off)
            hb = f.read(32)
            if len(hb) < 32:
                break
            magic, = struct.unpack_from("<I", hb, 0)
            if magic != REC_MAGIC:
                padding += 1
                continue
            seq, = struct.unpack_from("<Q", hb, 4)
            t_mono, = struct.unpack_from("<d", hb, 12)
            t_wall, = struct.unpack_from("<d", hb, 20)
            frames.append((off + rechdr, seq, t_mono, t_wall))
            if not quiet and i and i % 20000 == 0:
                print(f"\r  indexing {i}/{total}...", end="", file=sys.stderr)
    if not quiet and total > 20000:
        print("\r" + " " * 40 + "\r", end="", file=sys.stderr)
    if not frames:
        raise SystemExit(f"{path}: no valid frames found")
    return frames, padding


def measured_fps(frames):
    """Median frame rate from the monotonic timestamps.

    The header's advertised_fps is what the camera claims, not what it delivered
    -- on these captures it has been off by 2x -- so playback uses the timestamps.
    """
    if len(frames) < 2:
        return 25.0
    dts = np.diff(np.array([f[2] for f in frames]))
    dts = dts[dts > 0]
    if dts.size == 0:
        return 25.0
    return float(1.0 / np.median(dts))


class Capture:
    """Random-access reader. One seek and one read per frame; nothing cached."""

    def __init__(self, path, quiet=False):
        self.path = path
        self.meta = rawrec_format(path)
        self.frames, self.padding = build_index(path, self.meta, quiet)
        self.w = self.meta["width"]
        self.h = self.meta["height"]
        self.bpp = self.meta["bpp"]
        self.dtype = np.uint8 if self.bpp == 1 else np.dtype("<u2")
        self.nbytes = self.meta["frame_bytes"]
        self.fps = measured_fps(self.frames)
        self._f = open(path, "rb")

    def __len__(self):
        return len(self.frames)

    def frame(self, i):
        """The raw sensor array for frame i. Never modified by the viewer."""
        off = self.frames[i][0]
        self._f.seek(off)
        buf = self._f.read(self.nbytes)
        if len(buf) < self.nbytes:
            raise IOError(f"short read at frame {i}")
        return np.frombuffer(buf, dtype=self.dtype).reshape(self.h, self.w)

    def stamp(self, i):
        _, seq, t_mono, t_wall = self.frames[i]
        return seq, t_mono, t_wall

    def close(self):
        self._f.close()


def to_display(frame, bpp):
    """8-bit view of a frame for the screen only.

    8-bit captures pass through untouched. 16-bit captures have no meaningful
    8-bit rendering without a window, so a 1-99 percentile stretch is applied --
    display only; the probe and PNG export still use the source array.
    """
    if bpp == 1:
        return frame, None
    lo, hi = np.percentile(frame, (1, 99))
    if hi <= lo:
        hi = lo + 1
    scaled = np.clip((frame.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255)
    return scaled.astype(np.uint8), (float(lo), float(hi))


def _envelope(vals, width):
    """Per-column (min, max) of vals binned down to width columns.

    Binning by extremes rather than by sampling is deliberate: a single-pixel
    gradient spike is exactly what a profile is for, and picking every Nth
    sample would step straight over it.
    """
    n = vals.size
    width = max(1, min(width, n))
    starts = np.linspace(0, n, width, endpoint=False).astype(np.intp)
    return np.minimum.reduceat(vals, starts), np.maximum.reduceat(vals, starts)


def _subplot(canvas, rect, vals, probe, title, color, span=None, marks=None):
    """One profile plot: filled min/max envelope, probe marker, axis labels.

    span=None auto-scales to the data. span=m pins the axis to +/-m, which is
    what the derivative plots use: a fixed scale is the only way successive
    frames are comparable by eye -- with autoscaling, a peak that grows makes
    the axis grow with it and nothing appears to change.

    The cost of a fixed axis is that data can leave the box. Values are clamped
    to the frame so a spike cannot bleed into the neighbouring plot, and every
    clamped column is ticked in red on the border it left through, with the true
    extreme printed in the title. Clipping is therefore always visible; it is
    never a silently flattened peak.
    """
    x0, y0, w, h = rect
    cv2.rectangle(canvas, (x0, y0), (x0 + w - 1, y0 + h - 1), (55, 55, 55), 1)

    if span is not None:
        lo, hi = -float(span), float(span)
    else:
        lo, hi = float(vals.min()), float(vals.max())
        if hi <= lo:
            hi = lo + 1.0

    pw = max(1, w - 2)
    mins, maxs = _envelope(vals, pw)
    plot_h = h - 3

    def ymap(v):
        t = np.clip((hi - v) / (hi - lo), 0.0, 1.0)
        return y0 + 1 + np.rint(t * plot_h).astype(np.int32)

    xs = np.arange(mins.size, dtype=np.int32) + x0 + 1
    top, bot = ymap(maxs), ymap(mins)
    poly = np.concatenate([np.stack([xs, top], 1),
                           np.stack([xs[::-1], bot[::-1]], 1)])
    cv2.fillPoly(canvas, [poly], color)

    over, under = maxs > hi, mins < lo
    if over.any():
        canvas[y0 + 1:y0 + 3, xs[over]] = (60, 60, 255)
    if under.any():
        canvas[y0 + h - 3:y0 + h - 1, xs[under]] = (60, 60, 255)

    if lo < 0.0 < hi:
        yz = int(ymap(np.float64(0.0)))
        cv2.line(canvas, (x0 + 1, yz), (x0 + w - 2, yz), (95, 95, 95), 1)

    # probe marker, mapped from source index into plot columns
    px = x0 + 1 + int(probe / max(1, vals.size) * mins.size)
    cv2.line(canvas, (px, y0 + 1), (px, y0 + h - 2), (80, 220, 80), 1)

    if marks is not None and len(marks):
        mx = (x0 + 1 + np.asarray(marks) / max(1, vals.size) * mins.size)
        for m in np.clip(mx.astype(np.int32), x0 + 1, x0 + w - 2):
            cv2.line(canvas, (m, y0 + 1), (m, y0 + 8), (255, 90, 255), 1)

    cv2.putText(canvas, title, (x0 + 4, y0 + 13), cv2.FONT_HERSHEY_SIMPLEX,
                0.38, (215, 215, 215), 1, cv2.LINE_AA)
    if over.any() or under.any():
        peak = float(np.abs(vals).max()) if lo < 0 < hi else float(vals.max())
        cv2.putText(canvas, f"peak {peak:+.0f}",
                    (x0 + 4 + 9 * len(title), y0 + 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (90, 90, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"{hi:.0f}", (x0 + w - 34, y0 + 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (130, 130, 130), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"{lo:.0f}", (x0 + w - 34, y0 + h - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (130, 130, 130), 1, cv2.LINE_AA)


def profile_panel(frame, cx, cy, w, h, grad_span=GRAD_SPAN, det=None):
    """Row and column cuts through (cx, cy) with their first derivatives.

    Taken from the full-resolution source array, so the slope shown is the
    sensor's gradient -- unaffected by zoom, colormap or the display scaler.
    """
    panel = np.full((h, w, 3), 22, np.uint8)
    if cx is None:
        cv2.putText(panel, "move the mouse over the image", (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 150), 1,
                    cv2.LINE_AA)
        return panel

    row = frame[cy, :].astype(np.float64)
    col = frame[:, cx].astype(np.float64)
    gx, gy = np.gradient(row), np.gradient(col)   # central differences
    dx, dy = gx[cx], gy[cy]
    mag = float(np.hypot(dx, dy))
    ang = float(np.degrees(np.arctan2(dy, dx)))

    WHITE, GREEN, AMBER, DIM = ((235, 235, 235), (110, 235, 110),
                                (90, 175, 245), (150, 150, 150))
    head = [
        (f"probe ({cx},{cy})   I = {frame[cy, cx]}", WHITE),
        (f"dI/dx {dx:+7.2f}   dI/dy {dy:+7.2f}", WHITE),
        (f"|grad| {mag:6.2f}   dir {ang:+7.1f} deg", WHITE),
    ]

    marks_r = marks_c = None
    if det is None:
        head.append(("detect off  (d)", DIM))
    elif not HAVE_DET:
        head.append(("thermal_detect.py not found", AMBER))
    else:
        (mc, mm, mx), dets, (pc, pm, pcr, is_det) = det
        ndet = len(dets)
        # Detections are intensity maxima, so they belong on the intensity
        # plots: their x on the row cut, their y on the column cut.
        marks_r = np.array([d[0] for d in dets], int)
        marks_c = np.array([d[1] for d in dets], int)
        head.append((f"contrast>={mc:g}  moat>={mm:g}  crowd<={mx}", WHITE))
        head.append((f"detections in frame: {ndet}", WHITE))
        ok = pc >= mc and pm >= mm and pcr <= mx
        why = ("DETECTED" if is_det else
               "ok here" if ok else
               "too dim: contrast %d" % pc if pc < mc else
               "not isolated: moat %d" % pm if pm < mm else
               "crowded: %d peaks near" % pcr)
        head.append((f"probe contrast {pc:5d}  moat {pm:5d}  crowd {pcr:4d}",
                     WHITE))
        head.append((f"  -> {why}", GREEN if (ok or is_det) else AMBER))

    for k, (text, colr) in enumerate(head):
        cv2.putText(panel, text, (10, 18 + 16 * k), cv2.FONT_HERSHEY_SIMPLEX,
                    0.40, colr, 1, cv2.LINE_AA)

    top = 18 + 16 * len(head)
    ph = (h - top - 6) // 4
    plots = [
        (row, f"row y={cy}  intensity", (120, 120, 120), None, marks_r),
        (gx, f"row y={cy}  dI/dx", (235, 170, 70), grad_span, None),
        (col, f"col x={cx}  intensity", (120, 120, 120), None, marks_c),
        (gy, f"col x={cx}  dI/dy", (110, 200, 245), grad_span, None),
    ]
    probes = [cx, cx, cy, cy]
    for k, ((vals, title, color, span, marks), pr) in enumerate(
            zip(plots, probes)):
        _subplot(panel, (6, top + k * ph, w - 12, ph - 4), vals, pr,
                 title, color, span, marks)
    return panel


class Viewer:
    # NB: this shadows any module constant of the same name inside the
    # class body -- keep module constants distinct from these names.
    WIN = "rawrec"

    def __init__(self, cap, start=0, maxw=1400, maxh=900, outdir=None,
                 panel_w=420, grad_span=GRAD_SPAN,
                 min_contrast=MIN_CONTRAST, min_moat=MIN_MOAT,
                 max_crowd=MAX_CROWD, mouse_scale=1.0):
        self.cap = cap
        self.i = max(0, min(start, len(cap) - 1))
        self.maxw, self.maxh = maxw, maxh
        self.outdir = outdir or os.path.dirname(os.path.abspath(cap.path))
        self.fit = True
        self.zoom = 1.0
        self.ox = self.oy = 0
        self.cmap = 0
        self.hud = True
        self.playing = False
        self.rate = cap.fps
        self.cursor = None          # (x, y) in IMAGE pixels, or None
        self.cursor_win = None      # (x, y) as the window reported it
        self.panel = True           # profile plots beside the image
        self.panel_w = panel_w
        self.grad_span = grad_span
        self.det_on = HAVE_DET
        self.min_contrast, self.min_moat = min_contrast, min_moat
        self.max_crowd = max_crowd
        self.mouse_scale = mouse_scale
        self.marks = 1              # 0 off, 1 all peaks, 2 coincidences only
        self.clean = False          # 'a': suppress every mark drawn on the image
        self._mask_key = None
        self._masks = None
        self._bars = False          # detection sliders created yet?
        self.locked = False         # freeze the probe to step frames at one point
        self._drag = None
        self._syncing = False       # guards the trackbar callback against feedback

    # -- geometry ---------------------------------------------------------
    def box_w(self):
        """Width left for the image once the panel has taken its share."""
        return max(240, self.maxw - (self.panel_w if self.panel else 0))

    def fit_zoom(self):
        return min(self.box_w() / self.cap.w, self.maxh / self.cap.h, 1.0)

    def cur_zoom(self):
        return self.fit_zoom() if self.fit else self.zoom

    def visible(self, z):
        """Source-pixel rect currently on screen, clamped to the frame."""
        vw = min(self.cap.w, int(np.ceil(self.box_w() / z)))
        vh = min(self.cap.h, int(np.ceil(self.maxh / z)))
        ox = int(np.clip(self.ox, 0, max(0, self.cap.w - vw)))
        oy = int(np.clip(self.oy, 0, max(0, self.cap.h - vh)))
        self.ox, self.oy = ox, oy
        return ox, oy, vw, vh

    def screen_to_src(self, x, y):
        """Window coordinates -> image pixel, or None if not over the image.

        The window is the image with the profile panel stitched to its right,
        so a pointer over the panel has a window x that is still a small number
        after dividing by the zoom -- it used to map to a real-looking pixel
        that the mouse was nowhere near. The displayed image extent is now
        checked explicitly, before any mapping happens.
        """
        x, y = x * self.mouse_scale, y * self.mouse_scale
        z = self.cur_zoom()
        ox, oy, vw, vh = self.visible(z)
        if not (0 <= x < round(vw * z) and 0 <= y < round(vh * z)):
            return None                       # over the panel, or off the image
        sx, sy = int(ox + x / z), int(oy + y / z)
        if 0 <= sx < self.cap.w and 0 <= sy < self.cap.h:
            return sx, sy
        return None

    # -- peak overlay -----------------------------------------------------
    def detections(self, frame):
        """Detections plus the top-hat and SCR planes, cached on the parameters
        so panning and zooming never recompute them."""
        key = (self.i, self.min_contrast, self.min_moat, self.max_crowd)
        if key != self._mask_key:
            contrast, moat, crowd, _ = features(frame)
            dets = detect(frame, self.min_contrast, self.min_moat,
                          self.max_crowd)
            self._masks = (dets, contrast, moat, crowd)
            self._mask_key = key
        return self._masks

    def draw_peaks(self, out, frame, z, ox, oy):
        """Ring every detection on the display copy. Never touches the source."""
        dets, _, _, _ = self.detections(frame)
        for n, (sx, sy, con, moat, crowd) in enumerate(dets):
            dx, dy = int((sx - ox) * z), int((sy - oy) * z)
            if not (0 <= dx < out.shape[1] and 0 <= dy < out.shape[0]):
                continue
            cv2.circle(out, (dx, dy), 9, (255, 80, 255), 1, cv2.LINE_AA)
            if self.marks == 1:
                cv2.putText(out, f"{sx},{sy} c{con} m{moat}", (dx + 12, dy + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 120, 255), 1,
                            cv2.LINE_AA)
        return (len(dets),)

    # -- rendering --------------------------------------------------------
    def render(self, frame):
        z = self.cur_zoom()
        ox, oy, vw, vh = self.visible(z)
        crop = frame[oy:oy + vh, ox:ox + vw]

        disp, stretch = to_display(crop, self.cap.bpp)
        # Nearest keeps one stored pixel one block when magnifying; area-average
        # when shrinking so a small hot target cannot fall between samples.
        interp = cv2.INTER_NEAREST if z >= 1.0 else cv2.INTER_AREA
        out = cv2.resize(disp, (max(1, round(vw * z)), max(1, round(vh * z))),
                         interpolation=interp)

        name, cm = COLORMAPS[self.cmap]
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR) if cm is None \
            else cv2.applyColorMap(out, cm)
        # 'a' suppresses annotation without touching zoom, pan or panel width,
        # so flicking it on and off holds the image perfectly still -- the point
        # is to see what a marker is covering, which a reflow would defeat.
        counts = None
        if self.marks and self.det_on and HAVE_DET and not self.clean:
            counts = self.draw_peaks(out, frame, z, ox, oy)
        if self.cursor and not self.clean:
            # Drawn from the mapped IMAGE pixel back to the screen. If this
            # cross does not sit under the pointer, the mapping is wrong and
            # you can see it immediately rather than trusting a number.
            px, py = int((self.cursor[0] - ox) * z), int((self.cursor[1] - oy) * z)
            if 0 <= px < out.shape[1] and 0 <= py < out.shape[0]:
                cv2.line(out, (px - 7, py), (px - 2, py), (90, 240, 90), 1)
                cv2.line(out, (px + 2, py), (px + 7, py), (90, 240, 90), 1)
                cv2.line(out, (px, py - 7), (px, py - 2), (90, 240, 90), 1)
                cv2.line(out, (px, py + 2), (px, py + 7), (90, 240, 90), 1)
        if self.hud and not self.clean:
            self.draw_hud(out, frame, z, name, stretch, interp, counts)
        if self.panel:
            cx, cy = self.cursor if self.cursor else (None, None)
            det = None
            # The probe stats below index the frame at the cursor, so this
            # whole block needs a cursor. Before the mouse first enters the
            # window there is none, and the panel says so on its own.
            if self.det_on and HAVE_DET and self.cursor is not None:
                dets, contrast, moat, crowd = self.detections(frame)
                is_det = any(abs(dx - cx) <= MERGE and abs(dy - cy) <= MERGE
                             for dx, dy, _, _, _ in dets)
                det = ((self.min_contrast, self.min_moat, self.max_crowd),
                       dets, (int(contrast[cy, cx]), int(moat[cy, cx]),
                              int(crowd[cy, cx]), is_det))
            out = np.hstack([out, profile_panel(frame, cx, cy, self.panel_w,
                                                out.shape[0], self.grad_span,
                                                det)])
        return out

    def draw_hud(self, out, frame, z, cmname, stretch, interp, counts=None):
        seq, t_mono, t_wall = self.cap.stamp(self.i)
        t0 = self.cap.frames[0][2]
        lines = [
            f"frame {self.i + 1}/{len(self.cap)}   seq {seq}   "
            f"t+{t_mono - t0:8.3f}s   {self.cap.w}x{self.cap.h} "
            f"{self.cap.bpp * 8}-bit",
            f"zoom {z:.2f}x {'fit' if self.fit else '1:1'} "
            f"({'nearest' if interp == cv2.INTER_NEAREST else 'area'})   "
            f"cmap {cmname}   {'PLAY %.1f fps' % self.rate if self.playing else 'paused'}",
        ]
        if self.cursor:
            x, y = self.cursor
            wtxt = (f"win({self.cursor_win[0]},{self.cursor_win[1]}) -> "
                    if self.cursor_win else "")
            lines.append(f"{wtxt}px({x},{y}) = {frame[y, x]}   [raw sensor value]")
        if counts:
            lines.append(f"detections: {counts[0]}   "
                         f"contrast>={self.min_contrast:g} "
                         f"crowd<={self.max_crowd}   (m)")
        if stretch:
            lines.append(f"display stretch {stretch[0]:.0f}-{stretch[1]:.0f} "
                         f"(16-bit, screen only)")

        pad, lh = 6, 18
        box = out[0:pad * 2 + lh * len(lines), 0:out.shape[1]]
        cv2.addWeighted(box, 0.35, np.zeros_like(box), 0.65, 0, box)
        for k, text in enumerate(lines):
            cv2.putText(out, text, (pad, pad + lh * (k + 1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1,
                        cv2.LINE_AA)

    # -- input ------------------------------------------------------------
    def on_mouse(self, event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drag = (x, y, self.ox, self.oy)
        elif event == cv2.EVENT_LBUTTONUP:
            self._drag = None
        elif event == cv2.EVENT_MOUSEMOVE:
            self.cursor_win = (x, y)
            if not self.locked:
                src = self.screen_to_src(x, y)
                if src is not None:
                    self.cursor = src
            if self._drag:
                x0, y0, ox0, oy0 = self._drag
                z = self.cur_zoom()
                self.ox = ox0 - int((x - x0) / z)
                self.oy = oy0 - int((y - y0) / z)

    def on_track(self, pos):
        if not self._syncing:
            self.i = max(0, min(pos, len(self.cap) - 1))

    # -- live parameter tuning --------------------------------------------
    def flags(self):
        """Current thresholds as the command line that reproduces them."""
        return (f"--min-contrast {self.min_contrast:g} "
                f"--min-moat {self.min_moat:g} --max-crowd {self.max_crowd}")

    def _on_contrast(self, v):
        if not self._syncing:
            self.min_contrast = float(v)

    def _on_moat(self, v):
        if not self._syncing:
            self.min_moat = float(v)

    def _on_crowd(self, v):
        if not self._syncing:
            self.max_crowd = max(0, v)

    def _sync_bars(self):
        """Push key-driven changes back onto the sliders, without the setters
        firing again and fighting the keys for control."""
        if not self._bars:
            return
        self._syncing = True
        cv2.setTrackbarPos("min_contrast", self.WIN, int(self.min_contrast))
        cv2.setTrackbarPos("min_moat", self.WIN, int(self.min_moat))
        cv2.setTrackbarPos("max_crowd", self.WIN, int(self.max_crowd))
        self._syncing = False

    def seek(self, i):
        self.i = int(np.clip(i, 0, len(self.cap) - 1))

    def save_png(self, frame):
        stem = os.path.splitext(os.path.basename(self.cap.path))[0]
        seq = self.cap.stamp(self.i)[0]
        path = os.path.join(self.outdir, f"{stem}_f{self.i + 1:06d}_seq{seq}.png")
        # Writes the source array, not the display copy: lossless and bit-exact.
        if cv2.imwrite(path, frame):
            print(f"saved {path}  ({self.cap.w}x{self.cap.h} "
                  f"{self.cap.bpp * 8}-bit, lossless)")
        else:
            print(f"could not write {path}", file=sys.stderr)

    def info(self):
        seq, t_mono, t_wall = self.cap.stamp(self.i)
        frame = self.cap.frame(self.i)
        import datetime
        wall = datetime.datetime.fromtimestamp(t_wall, datetime.timezone.utc)
        print(f"frame {self.i + 1}/{len(self.cap)}  seq {seq}  "
              f"t_mono {t_mono:.6f}  t_wall {wall:%Y-%m-%d %H:%M:%S.%f} UTC")
        print(f"  min {frame.min()}  max {frame.max()}  mean {frame.mean():.2f}  "
              f"std {frame.std():.2f}  offset {self.cap.frames[self.i][0]}")

    # -- loop -------------------------------------------------------------
    def run(self):
        cv2.namedWindow(self.WIN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.WIN, self.on_mouse)
        cv2.createTrackbar("frame", self.WIN, self.i, len(self.cap) - 1,
                           self.on_track)
        if HAVE_DET:
            cv2.createTrackbar("min_contrast", self.WIN,
                               int(self.min_contrast), 200, self._on_contrast)
            cv2.createTrackbar("min_moat", self.WIN, int(self.min_moat), 80,
                               self._on_moat)
            cv2.createTrackbar("max_crowd", self.WIN, int(self.max_crowd), 200,
                               self._on_crowd)
            self._bars = True
            print(f"tuning: {self.flags()}")
        last = -1
        while True:
            frame = self.cap.frame(self.i)
            cv2.imshow(self.WIN, self.render(frame))
            if self.i != last:
                self._syncing = True
                cv2.setTrackbarPos("frame", self.WIN, self.i)
                self._syncing = False
                last = self.i

            delay = max(1, int(1000 / self.rate)) if self.playing else 30
            k = cv2.waitKeyEx(delay)
            if k == -1:
                if self.playing:
                    if self.i >= len(self.cap) - 1:
                        self.playing = False
                    else:
                        self.i += 1
                if cv2.getWindowProperty(self.WIN, cv2.WND_PROP_VISIBLE) < 1:
                    break
                continue

            c = k & 0xFF
            if k in (27, ord('q')):
                break
            elif k in (63235, 0x270000) or c in (ord('n'), ord('.')):
                self.seek(self.i + 1)
            elif k in (63234, 0x250000) or c in (ord('p'), ord(',')):
                self.seek(self.i - 1)
            elif c == ord('N') or k == 63236:
                self.seek(self.i + 10)
            elif c == ord('P') or k == 63237:
                self.seek(self.i - 10)
            elif k in (63232, 0x260000):
                self.seek(self.i + 100)
            elif k in (63233, 0x280000):
                self.seek(self.i - 100)
            elif k == 63273 or c == ord('0'):
                self.seek(0)
            elif k == 63275:
                self.seek(len(self.cap) - 1)
            elif c == ord(' '):
                self.playing = not self.playing
            elif c == ord(']'):
                self.rate = min(240.0, self.rate * 1.5)
            elif c == ord('['):
                self.rate = max(1.0, self.rate / 1.5)
            elif c == ord('z'):
                self.fit = not self.fit
                if not self.fit:
                    self.zoom = 1.0
            elif c in (ord('+'), ord('=')):
                self.fit = False
                self.zoom = min(16.0, self.zoom * 1.5)
            elif c in (ord('-'), ord('_')):
                self.fit = False
                self.zoom = max(0.1, self.zoom / 1.5)
            elif c == ord('c'):
                self.cmap = (self.cmap + 1) % len(COLORMAPS)
            elif c == ord('h'):
                self.hud = not self.hud
            elif c == ord('v'):
                self.panel = not self.panel
            elif c == ord('a'):
                self.clean = not self.clean
            elif c == ord('m'):
                self.marks = (self.marks + 1) % 3
            elif c == ord('d'):
                self.det_on = not self.det_on and HAVE_DET
                if not HAVE_DET:
                    print("needs thermal_detect.py", file=sys.stderr)
            elif c in (ord('e'), ord('r')):
                self.min_contrast = max(0.0, self.min_contrast
                                        + (5 if c == ord('r') else -5))
                self._sync_bars(); print(f"tuning: {self.flags()}")
            elif c in (ord('t'), ord('y')):
                self.max_crowd = max(0, self.max_crowd
                                     + (5 if c == ord('y') else -5))
                self._sync_bars(); print(f"tuning: {self.flags()}")
            elif c == ord('k'):
                print(f"tuning: {self.flags()}")
            elif c == ord('K'):
                self.min_contrast, self.min_moat = MIN_CONTRAST, MIN_MOAT
                self.max_crowd = MAX_CROWD
                self._sync_bars()
                print(f"tuning: {self.flags()}   (defaults)")
            elif c == ord('l'):
                self.locked = not self.locked
                print("probe " + ("locked at %s" % (self.cursor,) if self.locked
                                  else "follows the mouse"))
            elif c == ord('i'):
                self.info()
            elif c == ord('s'):
                self.save_png(frame)
            elif c == ord('g'):
                try:
                    self.seek(int(input(f"frame [1-{len(self.cap)}]: ")) - 1)
                except (ValueError, EOFError):
                    print("not a frame number", file=sys.stderr)
        cv2.destroyAllWindows()


def print_info(cap):
    seq0, t0, w0 = cap.frames[0][1:], cap.frames[0][2], cap.frames[0][3]
    tN = cap.frames[-1][2]
    dur = tN - cap.frames[0][2]
    print(f"file          {cap.path}")
    print(f"geometry      {cap.w}x{cap.h}  {cap.bpp * 8}-bit  "
          f"{cap.meta.get('pixfmt')}")
    print(f"frames        {len(cap)}  (+{cap.padding} zero-filled padding records)")
    print(f"duration      {dur:.3f} s")
    print(f"measured fps  {cap.fps:.4f}   (header advertises "
          f"{cap.meta.get('advertised_fps')})")
    seqs = np.array([f[1] for f in cap.frames])
    gaps = np.count_nonzero(np.diff(seqs) != 1)
    print(f"sequence      {seqs[0]}..{seqs[-1]}, {gaps} gap(s)")
    dts = np.diff(np.array([f[2] for f in cap.frames]))
    if dts.size:
        print(f"frame dt      median {np.median(dts) * 1e3:.2f} ms  "
              f"max {dts.max() * 1e3:.2f} ms")
    for k in ("focal", "episode", "source", "out_csv"):
        if k in cap.meta:
            print(f"{k:<14}{cap.meta[k]}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="the .rawrec capture to open")
    ap.add_argument("-f", "--frame", type=int, default=1,
                    help="open at this 1-based frame number")
    ap.add_argument("--info", action="store_true",
                    help="print header and index summary, then exit")
    ap.add_argument("--probe", type=int, metavar="N",
                    help="print stats for frame N, then exit (no GUI)")
    ap.add_argument("--outdir", help="where 's' writes PNGs "
                                     "(default: next to the capture)")
    ap.add_argument("--max-size", default="1400x900", metavar="WxH",
                    help="largest window, image plus profile panel")
    ap.add_argument("--panel-width", type=int, default=420, metavar="PX",
                    help="width of the profile panel (0 starts with it hidden)")
    ap.add_argument("--min-contrast", type=float, default=MIN_CONTRAST,
                    metavar="DN", help=f"minimum contrast above the local "
                                       f"median (default {MIN_CONTRAST:g})")
    ap.add_argument("--min-moat", type=float, default=MIN_MOAT, metavar="DN",
                    help=f"minimum margin over the brightest ring pixel; the "
                         f"isolation test (default {MIN_MOAT:g})")
    ap.add_argument("--max-crowd", type=int, default=MAX_CROWD, metavar="N",
                    help=f"most other peaks allowed nearby "
                         f"(default {MAX_CROWD})")
    ap.add_argument("--mouse-scale", type=float, default=1.0, metavar="F",
                    help="multiply incoming mouse coordinates by F; use 2.0 if "
                         "a HiDPI window reports points where the image has "
                         "pixels (the on-image crosshair shows the mismatch)")
    ap.add_argument("--no-detect", action="store_true",
                    help="start with the peak test off")
    ap.add_argument("--grad-scale", type=float, default=GRAD_SPAN, metavar="DN",
                    help="fixed +/- range of the derivative plots "
                         f"(default {GRAD_SPAN:g}; excursions are clipped and "
                         "ticked red)")
    args = ap.parse_args()

    cap = Capture(args.file, quiet=args.info or args.probe is not None)

    if args.info:
        print_info(cap)
        return
    if args.probe is not None:
        i = args.probe - 1
        if not 0 <= i < len(cap):
            raise SystemExit(f"frame {args.probe} out of range 1..{len(cap)}")
        v = Viewer(cap, i)
        v.info()
        return

    try:
        mw, mh = (int(v) for v in args.max_size.lower().split("x"))
    except ValueError:
        raise SystemExit(f"--max-size wants WxH, got {args.max_size!r}")

    print_info(cap)
    print("\nkeys: arrows/n/p step, space play, z zoom, c colormap, "
          "v profiles, l lock probe, s save PNG, i info, q quit")
    print("      d detect on/off, m overlay, e/r min_contrast -/+, "
          "t/y max_crowd -/+")
    print("      sliders tune min_contrast/min_moat/max_crowd live; "
          "k echoes them as flags, K resets")
    print("      a shows the raw image with no marks at all")
    v = Viewer(cap, args.frame - 1, mw, mh, args.outdir,
               abs(args.panel_width) or 420, args.grad_scale,
               args.min_contrast, args.min_moat, args.max_crowd,
               args.mouse_scale)
    v.panel = args.panel_width > 0
    if args.no_detect:
        v.det_on = False
    if not HAVE_DET:
        print("note: thermal_detect.py not found -- detection disabled")
    v.run()
    cap.close()


if __name__ == "__main__":
    main()
