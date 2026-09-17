"""Acquire the detection nearest a click on the live preview stream.

Rule: the operator watches http://<pi>:8002/ (flight/preview.py), clicks on
the target in the image, and the click's pixel -- already mapped back from
the encoded JPEG's scale to the camera's native frame -- arrives here as
context["manual_point"]. Take the detection whose centre is closest to it,
provided it is within MANUAL_CLICK_MAX_PX. No pending click, or nothing near
it, means no acquisition this frame -- which is the correct answer, not a
failure, exactly as in cue_nearest.

WHY THIS EXISTS ALONGSIDE cue_nearest, NOT INSTEAD OF IT. The radar cue is
what flies operationally: it needs no one watching a screen. This is the
manual fallback for a bench check, a sortie with no radar cue at all, or a
scene where the automatic rule (nearest-to-cue, or rightmost-isolated) picks
the wrong blob out of a clutter field and a human can tell at a glance which
one is real. Select it with --initialiser manual_click (or THERMAL_INITIALISER
in /etc/default/thermal-live) -- it does not replace FLIGHT_INITIALISER's
default.

WHAT THIS DELIBERATELY DOES NOT DO. Same as cue_nearest: it does not move,
weight, or filter any detection, and it does not substitute the click for
one. The output is always a real detector centroid or nothing -- so a click
that lands next to clutter acquires the clutter, not a phantom point, which
is a failure you can see and re-click rather than one that flies quietly.

CONTEXT KEYS, supplied by the caller (tools/flight_pipeline.py):
    manual_point   (u, v) pixel of the operator's most recent click, in the
                   camera's native frame coordinates, or None when no click is
                   pending (none was ever made, it was already consumed by a
                   previous acquisition, or it aged out -- see
                   flight/preview.py:Preview.click_point and
                   config.MANUAL_CLICK_TIMEOUT_S).
    manual_clear   callable, consumes the pending click so the SAME click
                   cannot re-acquire again after a later drop. Optional: a
                   caller that has no preview (e.g. an offline viewer) simply
                   omits it.
    frame_w/h      used only to reject a click that lands outside the image
"""
import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

MAX_DIST_PX = config.MANUAL_CLICK_MAX_PX


def init(boxes, context=None):
    """Return (x, y) of the detection nearest the pending click, or None."""
    if not boxes or not context:
        return None
    click = context.get("manual_point")
    if click is None:
        return None
    cu, cv = click
    w, h = context.get("frame_w"), context.get("frame_h")
    if w is not None and h is not None and not (0 <= cu < w and 0 <= cv < h):
        return None          # the click is off-image somehow; nothing to match

    best, best_d = None, None
    for box in boxes:
        x, y, bw, bh = box[:4]
        bx, by = x + bw / 2.0, y + bh / 2.0
        d = ((bx - cu) ** 2 + (by - cv) ** 2) ** 0.5
        if best_d is None or d < best_d:
            best, best_d = (bx, by), d
    if best_d is None or best_d > MAX_DIST_PX:
        return None

    clear = context.get("manual_clear")
    if callable(clear):
        clear()               # this click is spent, matched or not re-tried
    return best
