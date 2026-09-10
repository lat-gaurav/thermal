"""Acquire the detection nearest the radar cue.

Rule: project GCS_TARGET_BEARING into pixel space, then take the detection whose
centre is closest to it, provided it is within CUE_ACQUIRE_MAX_PX. No cue, or
nothing near it, means no acquisition this frame -- which is the correct answer,
not a failure.

WHY THIS REPLACES rightmost_isolated FOR FLIGHT. That one picks the rightmost
spatially-isolated detection, on the assumption that sky is on the right and a
distant drone shows up there alone. Against a detector that emits ~13 candidates
per frame on real flight footage, it acquires clutter on the first frame and --
because the tracker never releases a reference -- holds it for the whole sortie.
Measured on a bench scene with no target present at all: it locked at frame 1
and reported valid=1 to guidance on 840 of 841 frames.

The cue turns acquisition from "guess which blob is the target" into "which blob
is where the radar says the target is", which is a question with an answer.

WHAT THIS DELIBERATELY DOES NOT DO. It does not move, weight, or filter any
detection, and it does not substitute the cue for one. The output is always a
real detector centroid or nothing. The cue only chooses BETWEEN candidates the
detector found on its own, so detection performance is exactly what it was --
and if the cue is wrong, the failure mode is "did not acquire", not "acquired
the cue's error".

CONTEXT KEYS, supplied by the caller:
    cue_point   (u, v) pixel the cue projects to, or None when there is no
                usable cue. A cue reading gcs_target_valid=0 must arrive here as
                None: its az/el fields are then exactly zero, and zero is a
                legal bearing (dead ahead), so passing it through would acquire
                whatever sits near the boresight.
    frame_w/h   used only to reject a cue that lands outside the image
"""
import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

MAX_DIST_PX = config.CUE_ACQUIRE_MAX_PX


def init(boxes, context=None):
    """Return (x, y) of the detection nearest the cue, or None."""
    if not boxes or not context:
        return None
    cue = context.get("cue_point")
    if cue is None:
        return None
    cu, cv = cue
    w, h = context.get("frame_w"), context.get("frame_h")
    if w is not None and h is not None and not (0 <= cu < w and 0 <= cv < h):
        return None          # the cue is off-image; nothing in frame is "near" it

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
