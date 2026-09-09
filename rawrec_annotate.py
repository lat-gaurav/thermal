#!/usr/bin/env python3
"""Label targets in a .rawrec capture, and score the detector against them.

Built because ground truth was being dictated a frame at a time. Click the
targets, mark the empty frames, save; then --report tells you what the detector
gets right and wrong over the whole labelled set instead of one frame.

The detector is deliberately absent from the labelling view. Drawing its output
next to what you are about to click would bias the labels towards whatever it
already finds, and a ground truth agreeing with the thing it is meant to judge
is worth nothing. --report is the only place the detector is touched, and it
is imported there rather than at the top of the file so that stays true.

    rawrec_annotate.py FILE.rawrec              label, autoloading FILE.labels.csv
    rawrec_annotate.py FILE.rawrec -f 596       start at a frame
    rawrec_annotate.py FILE.rawrec --report     score the detector, no GUI
    rawrec_annotate.py FILE.rawrec --report -v  ...and list every disagreement

MOUSE

    left click         add a target here          middle drag   pan
    shift + left       add a REJECT here          scroll        (use +/-)
    right click        delete the label you clicked on

KEYS

    n / p / arrows     step 1 frame           N / P         step 10
    space              mark frame EMPTY and go to the next
    u                  undo the last edit     D             clear this frame
    f / F              accept the target / reject suggestion, then advance
    T / R              track targets / rejects forward until the match weakens
    z                  fit <-> 1:1            + / -         zoom
    c                  cycle colormap         [ / ]         marker smaller/bigger
    j                  jump to frame          , / .         prev/next LABELLED frame
    w                  write the CSV          q             quit (offers to write)

TRACKER.  Once a frame carries a target, the next unlabelled frame shows a cyan
diamond where the same object probably is, with its correlation score. The patch
you already labelled is matched by normalised cross-correlation inside a window
around a constant-velocity prediction of where the object is heading -- these
targets move several pixels per frame, so searching where it was is not enough.
It works even when the detector misses the object, which is when the help is
worth most.

A suggestion is only 'strong' (cyan) when the correlation clears MIN_NCC AND the
match stands out from its surroundings by MIN_CONTRAST. Both are needed: NCC is
scale-invariant, and a flat patch of sky correlates with a flat patch of sky at
0.6, well over the threshold. Weak suggestions are drawn dimmer with their
score, so you can still take one if you can see it is right.

Rejects are tracked too, on the same machinery: a cyan diamond is a target
suggestion, an orange one a reject. Clutter that fooled you once will fool you
again next frame, and a reject followed through its whole pass constrains the
detector far harder than a single isolated one.

'f'/'F' accept the target/reject suggestion and step on; 'T'/'R' keep accepting
that kind only while both numbers hold, then stop and say which one failed. A
reject is recorded exactly where the tracker put it -- unlike a target it marks
a place rather than an object, so snapping it to the nearest bright pixel could
slide it onto something else. Suggestions are never recorded on their own:
a tracker that drifted onto cloud would quietly poison the ground truth, so
every accepted row is printed and 'u' undoes them one at a time.

LABELS.  A row is frame,seq,x,y,label with label one of:

    target   a real object at (x, y) -- the detector SHOULD fire here
    reject   a specific place it must NOT fire
    empty    the whole frame contains nothing (x, y are -1)

'empty' matters: without it an unvisited frame and a frame you checked and found
clean look identical, and only the second is evidence. --report scores frames
that carry at least one row and ignores the rest, so partial labelling is safe.

MARKERS are hollow and leave the centre clear -- a green circle for a target, a
gapped X for a reject, a diamond for a tracker suggestion. Nothing is drawn on
the pixel itself: the objects are 2-4 px across, so a marker through the middle
hides exactly what you are trying to judge. Radii are in source pixels and scale with
zoom, so a marker keeps surrounding the object instead of shrinking onto it as
you magnify; a floor keeps it visible when zoomed out. '[' and ']' resize.

CLICKS SNAP to the brightest pixel within SNAP px, so two people clicking the
same object record the same coordinate. Shift-click to place a label exactly
where you clicked instead.
"""

import argparse
import csv
import os
import sys

import cv2
import numpy as np

from rawrec_view import Capture, COLORMAPS

# thermal_detect is imported lazily inside report(), never at module scope:
# labelling must not depend on the detector, and must not be tempted by it.

SNAP = 3            # click snaps to the brightest pixel within this radius
MATCH_R = 8         # a detection this close to a target counts as finding it
TPL_R = 7           # half-size of the appearance template
SEARCH_R = 40       # how far from the predicted position to look
MIN_NCC = 0.45      # below this a suggestion is shown but flagged weak
# NCC is scale-invariant, so it will happily match noise to noise: measured 0.61
# on a flat patch of sky, comfortably over MIN_NCC. Correlation alone is
# therefore not enough to accept a suggestion -- the match must also actually
# stand out from its surroundings. True targets measured +55..+186 DN.
MIN_CONTRAST = 20.0

# Marker geometry. Every marker is hollow with a clear central gap: these
# objects are 2-4 px across, so anything drawn through the middle hides the very
# thing you are trying to judge. Nothing is ever drawn at the centre.
#
# The radius is in SOURCE pixels and scales with zoom, so a marker always
# encircles the object rather than landing on top of it -- a screen-fixed radius
# looks fine zoomed out and then sits inside the magnified object when you zoom
# in to inspect it. MARKER_MIN_PX keeps it visible when zoomed out, where the
# scaled radius would otherwise collapse to a couple of pixels.
MARKER_R = 8        # outer radius in SOURCE px
MARKER_MIN_PX = 13  # never smaller than this on screen
GAP_FRAC = 0.45     # inner clear radius, as a fraction of the outer


def labels_path(cap_path, given=None):
    if given:
        return given
    return os.path.splitext(cap_path)[0] + ".labels.csv"


def load_labels(path):
    """{frame_index: [(x, y, label), ...]}. Missing file is not an error."""
    out = {}
    if not os.path.exists(path):
        return out
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                i = int(row["frame"]) - 1          # stored 1-based, like the HUD
                x, y = int(row["x"]), int(row["y"])
            except (KeyError, ValueError):
                continue
            lab = (row.get("label") or "target").strip()
            out.setdefault(i, []).append((x, y, lab))
    return out


def save_labels(path, labels, cap):
    """Write atomically -- a crash mid-write must not lose an afternoon."""
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "seq", "x", "y", "label"])
        for i in sorted(labels):
            for x, y, lab in labels[i]:
                w.writerow([i + 1, cap.stamp(i)[0], x, y, lab])
    os.replace(tmp, path)
    n = sum(len(v) for v in labels.values())
    print(f"wrote {path}  ({n} rows over {len(labels)} frames)")


def suggest(cap, labels, i, kind="target", search_r=SEARCH_R,
            tpl_r=TPL_R):
    """Where the thing labelled `kind` in an earlier frame probably is now.

    Works for rejects as well as targets. A piece of clutter that fooled you
    once will fool you again in the next frame, so tracking it is worth exactly
    as much as tracking a target -- and a reject followed through its whole pass
    is a far stronger constraint on the detector than a single isolated one.

    A correlation tracker, deliberately: the question is "the same object, about
    here", and normalised cross-correlation against the patch you already
    labelled answers exactly that. It also keeps working when the detector
    misses the object entirely, which is when help is most wanted.

    Position is predicted from the last two labels so a moving object is
    searched where it is going, not where it was -- these targets travel several
    pixels per frame.

    Returns (x, y, ncc, contrast) or None. Contrast is reported because NCC on
    its own is not trustworthy here: normalised correlation is scale-invariant,
    so featureless sky correlates with featureless sky at 0.6+. Both numbers
    have to be convincing before a suggestion is worth recording.
    """
    prev = [j for j in sorted(labels) if j < i
            and any(l == kind for _, _, l in labels[j])]
    if not prev:
        return None
    j = prev[-1]
    if i - j > 30:                       # too stale to be "about the same region"
        return None

    def tgt(k):
        for x, y, l in labels[k]:
            if l == kind:
                return x, y
        return None

    last = tgt(j)
    px, py = last
    if len(prev) >= 2:                   # constant-velocity extrapolation
        j2, p2 = prev[-2], tgt(prev[-2])
        if p2 and j != j2:
            vx, vy = (last[0] - p2[0]) / (j - j2), (last[1] - p2[1]) / (j - j2)
            px = int(round(last[0] + vx * (i - j)))
            py = int(round(last[1] + vy * (i - j)))

    ref = cap.frame(j)
    x0, x1 = max(0, last[0] - tpl_r), min(cap.w, last[0] + tpl_r + 1)
    y0, y1 = max(0, last[1] - tpl_r), min(cap.h, last[1] + tpl_r + 1)
    tpl = ref[y0:y1, x0:x1]
    if tpl.shape[0] < 3 or tpl.shape[1] < 3:
        return None

    cur = cap.frame(i)
    sx0, sx1 = max(0, px - search_r - tpl_r), min(cap.w, px + search_r + tpl_r + 1)
    sy0, sy1 = max(0, py - search_r - tpl_r), min(cap.h, py + search_r + tpl_r + 1)
    roi = cur[sy0:sy1, sx0:sx1]
    if roi.shape[0] < tpl.shape[0] or roi.shape[1] < tpl.shape[1]:
        return None
    res = cv2.matchTemplate(roi, tpl, cv2.TM_CCOEFF_NORMED)
    _, mx, _, loc = cv2.minMaxLoc(res)
    bx = sx0 + loc[0] + tpl.shape[1] // 2
    by = sy0 + loc[1] + tpl.shape[0] // 2
    # does the match actually stand out, or is it noise matching noise?
    r = 10
    around = cur[max(0, by - r):by + r + 1, max(0, bx - r):bx + r + 1]
    contrast = float(int(cur[by, bx]) - float(np.median(around)))
    return bx, by, float(mx), contrast


# ---------------------------------------------------------------- reporting
def report(cap, labels, match_r=MATCH_R, verbose=False, **kw):
    """Score detect_targets against the labelled frames."""
    frames = sorted(labels)
    if not frames:
        raise SystemExit("no labels yet -- run without --report and click some")

    # the detector; only --report touches it, never the labelling path
    from thermal_detect import detect as detect_targets

    hit = miss = fa = 0
    on_reject = 0
    empty_clean = empty_dirty = 0
    problems = []
    for i in frames:
        rows = labels[i]
        targets = [(x, y) for x, y, l in rows if l == "target"]
        rejects = [(x, y) for x, y, l in rows if l == "reject"]
        is_empty = any(l == "empty" for _, _, l in rows)
        dets = [(x, y) for x, y, *_ in detect_targets(cap.frame(i), **kw)]

        used = set()
        for tx, ty in targets:
            near = [k for k, (dx, dy) in enumerate(dets)
                    if k not in used and np.hypot(dx - tx, dy - ty) <= match_r]
            if near:
                used.add(near[0]); hit += 1
            else:
                miss += 1
                problems.append((i, "MISS", tx, ty))
        for k, (dx, dy) in enumerate(dets):
            if k in used:
                continue
            fa += 1
            tag = "FA"
            if any(np.hypot(dx - rx, dy - ry) <= match_r for rx, ry in rejects):
                on_reject += 1; tag = "FA-on-reject"
            problems.append((i, tag, dx, dy))
        if is_empty:
            if dets:
                empty_dirty += 1
            else:
                empty_clean += 1

    ntgt = hit + miss
    print(f"labelled frames   {len(frames)}")
    print(f"targets           {ntgt}")
    if ntgt:
        print(f"  found           {hit}  ({100 * hit / ntgt:.1f}% recall)")
        print(f"  missed          {miss}")
    print(f"false alarms      {fa}"
          + (f"   ({on_reject} on an explicit reject)" if on_reject else ""))
    if hit + fa:
        print(f"precision         {100 * hit / (hit + fa):.1f}%")
    ne = empty_clean + empty_dirty
    if ne:
        print(f"empty frames      {ne}: {empty_clean} clean, "
              f"{empty_dirty} with at least one detection")
    if verbose and problems:
        print("\ndisagreements:")
        for i, tag, x, y in problems:
            print(f"  frame {i + 1:6d}  {tag:13s} ({x},{y})")
    elif problems:
        print(f"\n{len(problems)} disagreement(s) -- add -v to list them")


# ---------------------------------------------------------------- the tool
class Annotator:
    WIN = "annotate"

    def __init__(self, cap, labels, path, start=0, maxw=1500, maxh=950):
        self.cap, self.labels, self.path = cap, labels, path
        self.i = max(0, min(start, len(cap) - 1))
        self.maxw, self.maxh = maxw, maxh
        self.fit, self.zoom = True, 1.0
        self.ox = self.oy = 0
        self.cmap = 0
        self.msize = MARKER_R
        self.dirty = False
        self.undo = []              # (frame, list-before) for one-step-per-edit
        self._drag = None
        self._suggcache = {}        # (frame, kind) -> suggestion
        self.search_r = SEARCH_R

    # -- geometry (same mapping discipline as the main viewer) ----------
    def cur_zoom(self):
        return min(self.maxw / self.cap.w, self.maxh / self.cap.h, 1.0) \
            if self.fit else self.zoom

    def visible(self, z):
        vw = min(self.cap.w, int(np.ceil(self.maxw / z)))
        vh = min(self.cap.h, int(np.ceil(self.maxh / z)))
        self.ox = int(np.clip(self.ox, 0, max(0, self.cap.w - vw)))
        self.oy = int(np.clip(self.oy, 0, max(0, self.cap.h - vh)))
        return self.ox, self.oy, vw, vh

    def to_src(self, x, y):
        z = self.cur_zoom()
        ox, oy, vw, vh = self.visible(z)
        if not (0 <= x < round(vw * z) and 0 <= y < round(vh * z)):
            return None
        sx, sy = int(ox + x / z), int(oy + y / z)
        return (sx, sy) if (0 <= sx < self.cap.w and 0 <= sy < self.cap.h) else None

    # -- labels ---------------------------------------------------------
    def rows(self):
        return self.labels.get(self.i, [])

    def _push(self):
        self.undo.append((self.i, list(self.rows())))
        del self.undo[:-200]
        self.dirty = True

    def add(self, x, y, lab, snap=True):
        if snap:
            f = self.cap.frame(self.i)
            r = SNAP
            sub = f[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1]
            oy, ox = np.unravel_index(sub.argmax(), sub.shape)
            x, y = max(0, x - r) + int(ox), max(0, y - r) + int(oy)
        self._push()
        rows = [r for r in self.rows() if r[2] != "empty"]   # no longer empty
        rows.append((x, y, lab))
        self.labels[self.i] = rows
        print(f"frame {self.i + 1}: +{lab} ({x},{y})")

    def delete_near(self, x, y):
        """Delete the label nearest the click, if the click was near enough.

        The tolerance is a SCREEN distance, not a source one: a fixed source
        radius is fiddly zoomed out and absurdly generous zoomed in, where 25
        source px can be a quarter of the window and would snatch a label you
        were nowhere near. Tied to the marker radius, so "click on the marker"
        means the same thing at every zoom.
        """
        rows = self.rows()
        cand = [(np.hypot(rx - x, ry - y), k)
                for k, (rx, ry, l) in enumerate(rows) if l != "empty"]
        if not cand:
            print(f"frame {self.i + 1}: nothing to delete here")
            return
        d, k = min(cand)
        z = self.cur_zoom()
        tol_screen = max(24, 2 * max(MARKER_MIN_PX, int(self.msize * z)))
        if d * z > tol_screen:
            print(f"frame {self.i + 1}: no label within reach "
                  f"(nearest is {d:.0f} px away, {len(cand)} in this frame) "
                  f"-- right-click closer, or D clears the frame")
            return
        self._push()
        rx, ry, lab = rows[k]
        self.labels[self.i] = rows[:k] + rows[k + 1:]
        if not self.labels[self.i]:
            del self.labels[self.i]
        print(f"frame {self.i + 1}: -{lab} ({rx},{ry})   [u restores it]")

    def mark_empty(self):
        self._push()
        self.labels[self.i] = [(-1, -1, "empty")]
        print(f"frame {self.i + 1}: EMPTY")

    def clear(self):
        if self.i in self.labels:
            self._push()
            del self.labels[self.i]
            print(f"frame {self.i + 1}: cleared")

    def pop_undo(self):
        if not self.undo:
            return
        i, rows = self.undo.pop()
        self.i = i
        if rows:
            self.labels[i] = rows
        else:
            self.labels.pop(i, None)
        self.dirty = True
        print(f"undo -> frame {i + 1}")

    def sugg(self, kind="target"):
        """Tracker suggestion of this kind, cached. None once the frame has one."""
        if any(l == kind for _, _, l in self.rows()):
            return None
        key = (self.i, kind)
        if key not in self._suggcache:
            self._suggcache = {key: suggest(self.cap, self.labels, self.i,
                                            kind, self.search_r)}
        return self._suggcache[key]

    def accept_sugg(self, kind="target"):
        """Manual accept. Warns on a weak match but does not refuse it -- you
        can see the frame, and your judgement beats the score."""
        s = self.sugg(kind)
        if not s:
            print(f"no {kind} suggestion here")
            return False
        x, y, ncc, con = s
        if ncc < MIN_NCC or con < MIN_CONTRAST:
            print(f"   note: weak match (ncc {ncc:.2f}, contrast {con:+.0f}) "
                  f"-- accepting anyway because you asked")
        # A reject marks a place, not an object, so it is recorded exactly where
        # the tracker put it; snapping could slide it onto a neighbouring peak.
        self.add(x, y, kind, snap=(kind == "target"))
        print(f"   accepted {kind} suggestion (ncc {ncc:.2f}, "
              f"contrast {con:+.0f})")
        return True

    def track_forward(self, kind="target", n=25, min_ncc=MIN_NCC):
        """Accept suggestions frame by frame until they stop being convincing.

        Bounded and printed rather than silent: these rows become ground truth,
        and a tracker that drifted onto cloud would poison it. Stops on the
        first weak match and says why; 'u' undoes one frame at a time.
        """
        n_done = 0
        for _ in range(n):
            if self.i >= len(self.cap) - 1:
                print("track: end of capture")
                break
            self.i += 1
            s = self.sugg(kind)
            if s is None:
                print(f"track: no {kind} suggestion at frame {self.i + 1}")
                break
            if s[2] < min_ncc:
                print(f"track: stopped at frame {self.i + 1}, "
                      f"ncc {s[2]:.2f} < {min_ncc}")
                break
            if s[3] < MIN_CONTRAST:
                print(f"track: stopped at frame {self.i + 1}, contrast "
                      f"{s[3]:+.0f} < {MIN_CONTRAST:g} -- correlation alone can "
                      f"lock onto featureless sky")
                break
            self.add(s[0], s[1], kind, snap=(kind == "target"))
            n_done += 1
        print(f"track: labelled {n_done} frame(s) as {kind}")

    def step_labelled(self, back=False):
        ks = sorted(self.labels)
        ks = [k for k in ks if (k < self.i if back else k > self.i)]
        if ks:
            self.i = ks[-1] if back else ks[0]

    # -- drawing --------------------------------------------------------
    def render(self):
        frame = self.cap.frame(self.i)
        z = self.cur_zoom()
        ox, oy, vw, vh = self.visible(z)
        crop = frame[oy:oy + vh, ox:ox + vw]
        interp = cv2.INTER_NEAREST if z >= 1.0 else cv2.INTER_AREA
        disp = cv2.resize(crop, (max(1, round(vw * z)), max(1, round(vh * z))),
                          interpolation=interp)
        name, cm = COLORMAPS[self.cmap]
        out = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR) if cm is None \
            else cv2.applyColorMap(disp, cm)

        def sp(x, y):
            return int((x - ox) * z), int((y - oy) * z)

        # source-pixel radius scaled to the screen, with a visibility floor
        R = max(MARKER_MIN_PX, int(self.msize * z))
        for kind, strongc, weakc, key in (
                ("target", (80, 230, 230), (80, 160, 200), "f"),
                ("reject", (60, 170, 255), (70, 130, 190), "F")):
            s = self.sugg(kind)
            if not s:
                continue
            dx, dy = sp(s[0], s[1])
            strong = s[2] >= MIN_NCC and s[3] >= MIN_CONTRAST
            col = strongc if strong else weakc
            d = int(R * 1.6)                      # hollow diamond, drawn wide
            pts = np.array([[dx, dy - d], [dx + d, dy], [dx, dy + d],
                            [dx - d, dy]], np.int32)
            cv2.polylines(out, [pts], True, col, 1, cv2.LINE_AA)
            cv2.putText(out, f"{kind[:3]} {s[2]:.2f} c{s[3]:+.0f} ({key})",
                        (dx + d + 5, dy + 4 + (0 if kind == "target" else 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

        is_empty = False
        for x, y, lab in self.rows():
            if lab == "empty":
                is_empty = True
                continue
            dx, dy = sp(x, y)
            g = int(R * GAP_FRAC)                 # clear radius around the pixel
            if lab == "target":
                cv2.circle(out, (dx, dy), R, (90, 240, 90), 2, cv2.LINE_AA)
            else:
                # An X with the middle cut out: the old one ran through the
                # centre and hid the object it was pointing at.
                for ux, uy in ((-1, -1), (1, 1), (-1, 1), (1, -1)):
                    a = (dx + int(ux * g * 0.71), dy + int(uy * g * 0.71))
                    b = (dx + int(ux * R * 0.85), dy + int(uy * R * 0.85))
                    cv2.line(out, a, b, (60, 60, 255), 2, cv2.LINE_AA)

        nt = sum(1 for _, _, l in self.rows() if l == "target")
        nr = sum(1 for _, _, l in self.rows() if l == "reject")
        state = "EMPTY" if is_empty else (f"{nt} target, {nr} reject"
                                          if (nt or nr) else "unlabelled")
        lines = [
            f"frame {self.i + 1}/{len(self.cap)}  seq {self.cap.stamp(self.i)[0]}"
            f"   [{state}]{'  *unsaved' if self.dirty else ''}",
            f"labelled {len(self.labels)} frames   "
            f"zoom {z:.2f} {'fit' if self.fit else '1:1'}  cmap {name}",
            "left=target  shift+left=reject  right=delete  space=empty  "
            "f/F=accept track  T/R=track fwd  w=write  q=quit",
        ]
        pad, lh = 6, 18
        box = out[0:pad * 2 + lh * len(lines), :]
        cv2.addWeighted(box, 0.3, np.zeros_like(box), 0.7, 0, box)
        for k, t in enumerate(lines):
            cv2.putText(out, t, (pad, pad + lh * (k + 1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1,
                        cv2.LINE_AA)
        return out

    # -- input ----------------------------------------------------------
    def on_mouse(self, ev, x, y, flags, _):
        if ev == cv2.EVENT_MBUTTONDOWN:
            self._drag = (x, y, self.ox, self.oy)
        elif ev == cv2.EVENT_MBUTTONUP:
            self._drag = None
        elif ev == cv2.EVENT_MOUSEMOVE and self._drag:
            x0, y0, ox0, oy0 = self._drag
            z = self.cur_zoom()
            self.ox = ox0 - int((x - x0) / z)
            self.oy = oy0 - int((y - y0) / z)
        elif ev == cv2.EVENT_LBUTTONDOWN:
            s = self.to_src(x, y)
            if s:
                shift = bool(flags & cv2.EVENT_FLAG_SHIFTKEY)
                self.add(s[0], s[1], "reject" if shift else "target",
                         snap=not shift)
        elif ev == cv2.EVENT_RBUTTONDOWN:
            s = self.to_src(x, y)
            if s:
                self.delete_near(*s)

    def run(self):
        cv2.namedWindow(self.WIN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.WIN, self.on_mouse)
        cv2.createTrackbar("frame", self.WIN, self.i, len(self.cap) - 1,
                           lambda v: setattr(self, "i", v))
        last = -1
        while True:
            cv2.imshow(self.WIN, self.render())
            if self.i != last:
                cv2.setTrackbarPos("frame", self.WIN, self.i)
                last = self.i
            k = cv2.waitKeyEx(30)
            if k == -1:
                if cv2.getWindowProperty(self.WIN, cv2.WND_PROP_VISIBLE) < 1:
                    break
                continue
            c = k & 0xFF
            if k in (27, ord('q')):
                if self.dirty:
                    save_labels(self.path, self.labels, self.cap)
                break
            elif k in (63235, 0x270000) or c == ord('n'):
                self.i = min(len(self.cap) - 1, self.i + 1)
            elif k in (63234, 0x250000) or c == ord('p'):
                self.i = max(0, self.i - 1)
            elif c == ord('N'):
                self.i = min(len(self.cap) - 1, self.i + 10)
            elif c == ord('P'):
                self.i = max(0, self.i - 10)
            elif c == ord(' '):
                self.mark_empty()
                self.i = min(len(self.cap) - 1, self.i + 1)
            elif c == ord('f'):
                if self.accept_sugg("target"):
                    self.i = min(len(self.cap) - 1, self.i + 1)
            elif c == ord('F'):
                if self.accept_sugg("reject"):
                    self.i = min(len(self.cap) - 1, self.i + 1)
            elif c == ord('T'):
                self.track_forward("target")
            elif c == ord('R'):
                self.track_forward("reject")
            elif c == ord('u'):
                self.pop_undo()
            elif c == ord('D'):
                self.clear()
            elif c == ord('z'):
                self.fit = not self.fit
                if not self.fit:
                    self.zoom = 1.0
            elif c in (ord('+'), ord('=')):
                self.fit = False; self.zoom = min(16.0, self.zoom * 1.5)
            elif c in (ord('-'), ord('_')):
                self.fit = False; self.zoom = max(0.1, self.zoom / 1.5)
            elif c == ord('c'):
                self.cmap = (self.cmap + 1) % len(COLORMAPS)
            elif c == ord(']'):
                self.msize = min(60, self.msize + 2)
                print(f"marker radius {self.msize} source px")
            elif c == ord('['):
                self.msize = max(2, self.msize - 2)
                print(f"marker radius {self.msize} source px")
            elif c == ord('.'):
                self.step_labelled()
            elif c == ord(','):
                self.step_labelled(back=True)
            elif c == ord('w'):
                save_labels(self.path, self.labels, self.cap)
                self.dirty = False
            elif c == ord('j'):
                try:
                    self.i = int(np.clip(
                        int(input(f"frame [1-{len(self.cap)}]: ")) - 1,
                        0, len(self.cap) - 1))
                except (ValueError, EOFError):
                    print("not a frame number", file=sys.stderr)
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("-f", "--frame", type=int, default=1)
    ap.add_argument("--labels", help="CSV to use (default FILE.labels.csv)")
    ap.add_argument("--report", action="store_true",
                    help="score the detector against the labels and exit")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="with --report, list every disagreement")
    ap.add_argument("--match-radius", type=int, default=MATCH_R, metavar="PX",
                    help=f"a detection within this counts as a hit "
                         f"(default {MATCH_R})")
    ap.add_argument("--marker-size", type=int, default=MARKER_R, metavar="PX",
                    help=f"marker radius in SOURCE px; scales with zoom, "
                         f"[ and ] change it live (default {MARKER_R})")
    ap.add_argument("--search-radius", type=int, default=SEARCH_R, metavar="PX",
                    help=f"how far the tracker looks from its prediction "
                         f"(default {SEARCH_R})")
    ap.add_argument("--max-size", default="1500x950", metavar="WxH")
    args = ap.parse_args()

    cap = Capture(args.file, quiet=True)
    path = labels_path(args.file, args.labels)
    labels = load_labels(path)
    print(f"{cap.w}x{cap.h}, {len(cap)} frames   labels: {path} "
          f"({len(labels)} frames already)")

    if args.report:
        report(cap, labels, args.match_radius, args.verbose)
        return
    try:
        mw, mh = (int(v) for v in args.max_size.lower().split("x"))
    except ValueError:
        raise SystemExit(f"--max-size wants WxH, got {args.max_size!r}")
    a = Annotator(cap, labels, path, args.frame - 1, mw, mh)
    a.search_r = args.search_radius
    a.msize = args.marker_size
    a.run()
    cap.close()


if __name__ == "__main__":
    main()
