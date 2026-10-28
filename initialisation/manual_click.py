"""Acquire the detection nearest a click on the live preview stream.

Rule: the operator watches http://<pi>:8002/ (flight/preview.py), clicks on
the target in the image, and the click's pixel -- already mapped back from
the encoded JPEG's scale to the camera's native frame -- arrives here as
context["manual_point"]. Take the detection whose centre is closest to it,
provided it is within MANUAL_CLICK_MAX_PX. No pending click, or nothing near
it, means no acquisition. Each click gets one attempt, matched or unmatched.
In the flight pipeline, a worker matches on the exact clicked historical frame
and replays a candidate forward before overriding the live lock. An unmatched
click leaves normal tracking intact. A manual lock is exempt from radar-disagreement release
while manual_click remains selected; the residual is still logged.

WHY THIS EXISTS ALONGSIDE cue_nearest, NOT INSTEAD OF IT. This is
FLIGHT_INITIALISER's default, so a sortie starts with acquisition under the
operator's hand: a bench check, a sortie with no radar cue at all, or a scene
where the automatic rule (nearest-to-cue, or rightmost-isolated) picks the
wrong blob out of a clutter field and a human can tell at a glance which one
is real. cue_nearest remains what flies operationally once the operator hands
acquisition back to it -- it needs no one watching a screen -- from the
preview's initialiser dropdown, --initialiser cue_nearest, or
THERMAL_INITIALISER in /etc/default/thermal-live.

WHAT THIS DELIBERATELY DOES NOT DO. Same as cue_nearest: it does not move,
weight, or filter any detection, and it does not substitute the click for
one. The output is always a real detector centroid or nothing -- so a click
that lands next to clutter acquires the clutter, not a phantom point, which
is a failure you can see and re-click rather than one that flies quietly.

CONTEXT KEYS, supplied by the caller (tools/flight_pipeline.py):
    manual_point   (u, v) pixel of the operator's most recent click, in the
                   camera's native frame coordinates, or None when no click is
                   pending (none was ever made, it was already consumed by a
                   previous attempt, or it aged out -- see
                   flight/preview.py:Preview.click_point and
                   config.MANUAL_CLICK_TIMEOUT_S).
    manual_clear   optional callable, consumes the click on ANY attempt. The
                   flight pipeline instead consumes atomically before detection
                   with Preview.take_click(), so a newer click is never erased.
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
    if not context:
        return None
    click = context.get("manual_point")
    if click is None:
        return None
    clear = context.get("manual_clear")
    if callable(clear):
        clear()               # one attempt, including an unmatched click
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

    return best
