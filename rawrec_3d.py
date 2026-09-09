#!/usr/bin/env python3
"""Show a .rawrec frame as a 3-D surface: intensity, or its gradient.

A profile through one row tells you about that row. A surface tells you whether
a bump is a peak or the shoulder of a ridge, which is usually the actual
question. Reads frames through rawrec_view.Capture, so the pixels are the
sensor's -- nothing is decoded or re-quantised on the way in.

    rawrec_3d.py FILE.rawrec -f 596                full frame, decimated
    rawrec_3d.py FILE.rawrec -f 596 --roi 606,623  full-resolution patch
    rawrec_3d.py FILE.rawrec -f 596 --save s.png   render headless and exit

KEYS

    n / p        next / previous frame     N / P    +/- 10 frames
    g            cycle surface             c        cycle colormap
    w            wireframe / solid         r        reset the viewpoint
    d            detection markers         s        save a PNG
    q            quit                      drag     rotate, scroll zoom

SURFACES.  intensity, |grad|, dI/dx, dI/dy. Gradients are computed at FULL
resolution and only then decimated, never the other way round -- differentiating
an already-decimated image would show the slope of a smaller picture, not the
sensor's.

DECIMATION.  A 1280x1024 surface is 1.3M facets and will not draw interactively,
so the full-frame view reduces by blocks. Intensity blocks are averaged, but
gradients take the block extreme (largest magnitude, sign kept): a one-pixel
gradient spike is the thing you are looking for, and averaging or sampling would
erase it. The title always states which reduction is in force, and --roi avoids
the question entirely by showing real pixels.
"""

import argparse
import sys

import numpy as np

try:
    from rawrec_view import Capture, detect_targets
    HAVE_DETECT = True
except ImportError as e:                       # pragma: no cover
    print(f"needs rawrec_view.py beside this script: {e}", file=sys.stderr)
    raise

SURFACES = ("intensity", "|grad|", "dI/dx", "dI/dy")
COLORMAPS = ("inferno", "viridis", "turbo", "gray")


def block_reduce(a, k, how):
    """Reduce a by k x k blocks. how: mean, max, or absmax (keeps sign)."""
    if k <= 1:
        return a
    h, w = a.shape
    h2, w2 = h // k * k, w // k * k
    b = a[:h2, :w2].reshape(h2 // k, k, w2 // k, k)
    if how == "mean":
        return b.mean(axis=(1, 3))
    if how == "max":
        return b.max(axis=(1, 3))
    mx, mn = b.max(axis=(1, 3)), b.min(axis=(1, 3))
    return np.where(np.abs(mx) >= np.abs(mn), mx, mn)


def surface_for(frame, kind):
    """The chosen surface at full resolution, plus how it should be reduced."""
    f = frame.astype(np.float64)
    if kind == "intensity":
        return f, "mean"
    if kind == "|grad|":
        gy, gx = np.gradient(f)
        return np.hypot(gx, gy), "max"
    if kind == "dI/dx":
        return np.gradient(f, axis=1), "absmax"
    return np.gradient(f, axis=0), "absmax"


class Surface3D:
    def __init__(self, cap, start=0, roi=None, size=64, stride=None,
                 detect=True):
        self.cap = cap
        self.i = max(0, min(start, len(cap) - 1))
        self.roi, self.size = roi, size
        self.kind = 0
        self.cmap = 0
        self.wire = False
        self.detect = detect and HAVE_DETECT
        # Aim at roughly 200 columns of facets; that draws in well under a
        # second, which is what makes rotating feel like rotating.
        self.stride = stride or max(1, cap.w // 200)
        self.view = None

    def data(self):
        """(X, Y, Z, label) for the current frame, surface and view."""
        frame = self.cap.frame(self.i)
        kind = SURFACES[self.kind]
        full, how = surface_for(frame, kind)

        if self.roi:
            cx, cy = self.roi
            h = self.size // 2
            x0, x1 = max(0, cx - h), min(self.cap.w, cx + h + 1)
            y0, y1 = max(0, cy - h), min(self.cap.h, cy + h + 1)
            Z = full[y0:y1, x0:x1]
            X, Y = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
            note = f"ROI {x1 - x0}x{y1 - y0} @({cx},{cy})  full resolution"
        else:
            k = self.stride
            Z = block_reduce(full, k, how)
            X, Y = np.meshgrid(np.arange(Z.shape[1]) * k,
                               np.arange(Z.shape[0]) * k)
            note = (f"full frame, {k}x{k} blocks reduced by "
                    f"{'average' if how == 'mean' else 'extreme'}")
        return X, Y, Z, note

    def markers(self):
        """Detections inside the current view, as (x, y) pairs."""
        if not self.detect:
            return []
        out = []
        for x, y, *_ in detect_targets(self.cap.frame(self.i)):
            if self.roi:
                cx, cy = self.roi
                h = self.size // 2
                if not (abs(x - cx) <= h and abs(y - cy) <= h):
                    continue
            out.append((x, y))
        return out

    def draw(self, ax, fig):
        if self.view is None:
            self.view = (ax.elev, ax.azim)
        else:                                  # keep the viewpoint across redraws
            self.view = (ax.elev, ax.azim)
        elev, azim = self.view
        ax.clear()

        X, Y, Z, note = self.data()
        kind = SURFACES[self.kind]
        cm = COLORMAPS[self.cmap]
        if self.wire:
            ax.plot_wireframe(X, Y, Z, rstride=1, cstride=1, linewidth=0.3,
                              color="0.4")
        else:
            ax.plot_surface(X, Y, Z, cmap=cm, linewidth=0, antialiased=False,
                            rstride=1, cstride=1)

        frame = self.cap.frame(self.i)
        full, _ = surface_for(frame, kind)
        for mx, my in self.markers():
            ax.scatter([mx], [my], [full[my, mx]], color="red", s=40,
                       depthshade=False)
            ax.text(mx, my, full[my, mx], f"  {mx},{my}", color="red",
                    fontsize=7)

        seq, t_mono, _ = self.cap.stamp(self.i)
        ax.set_title(f"frame {self.i + 1}/{len(self.cap)}  seq {seq}   "
                     f"{kind}   [{note}]\n"
                     f"z range {Z.min():.0f}..{Z.max():.0f}"
                     f"   keys: n/p g c w r d s q",
                     fontsize=9)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel(kind)
        ax.view_init(elev=elev, azim=azim)
        fig.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="the .rawrec capture")
    ap.add_argument("-f", "--frame", type=int, default=1,
                    help="1-based frame number")
    ap.add_argument("--roi", metavar="X,Y",
                    help="centre a full-resolution patch here instead of "
                         "showing the whole decimated frame")
    ap.add_argument("--size", type=int, default=64,
                    help="side of the --roi patch in pixels (default 64)")
    ap.add_argument("--stride", type=int,
                    help="block size for the full-frame view "
                         "(default: about 200 columns of facets)")
    ap.add_argument("--surface", choices=SURFACES, default="intensity")
    ap.add_argument("--no-detect", action="store_true",
                    help="do not mark detections")
    ap.add_argument("--save", metavar="PNG",
                    help="render once to this file and exit, no window")
    args = ap.parse_args()

    cap = Capture(args.file, quiet=True)
    roi = None
    if args.roi:
        try:
            rx, ry = (int(v) for v in args.roi.split(","))
        except ValueError:
            raise SystemExit(f"--roi wants X,Y, got {args.roi!r}")
        if not (0 <= rx < cap.w and 0 <= ry < cap.h):
            raise SystemExit(f"--roi {rx},{ry} outside {cap.w}x{cap.h}")
        roi = (rx, ry)

    import matplotlib
    if args.save:
        matplotlib.use("agg")
    import matplotlib.pyplot as plt

    s = Surface3D(cap, args.frame - 1, roi, args.size, args.stride,
                  not args.no_detect)
    s.kind = SURFACES.index(args.surface)

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    s.draw(ax, fig)

    if args.save:
        fig.savefig(args.save, dpi=110, bbox_inches="tight")
        print(f"saved {args.save}")
        return

    def on_key(ev):
        k = ev.key
        if k == "q":
            plt.close(fig); return
        elif k == "n":
            s.i = min(len(cap) - 1, s.i + 1)
        elif k == "p":
            s.i = max(0, s.i - 1)
        elif k == "N":
            s.i = min(len(cap) - 1, s.i + 10)
        elif k == "P":
            s.i = max(0, s.i - 10)
        elif k == "g":
            s.kind = (s.kind + 1) % len(SURFACES)
        elif k == "c":
            s.cmap = (s.cmap + 1) % len(COLORMAPS)
        elif k == "w":
            s.wire = not s.wire
        elif k == "d":
            s.detect = not s.detect and HAVE_DETECT
        elif k == "r":
            s.view = (30.0, -60.0)
            ax.view_init(30, -60)
        elif k == "s":
            name = f"rawrec3d_f{s.i + 1:06d}_{SURFACES[s.kind].replace('/', '')}.png"
            fig.savefig(name, dpi=110, bbox_inches="tight")
            print(f"saved {name}"); return
        else:
            return
        s.draw(ax, fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    print(f"{cap.w}x{cap.h}, {len(cap)} frames. "
          f"keys: n/p frame, g surface, c colormap, w wireframe, "
          f"d markers, r reset, s save, q quit")
    plt.show()


if __name__ == "__main__":
    main()
