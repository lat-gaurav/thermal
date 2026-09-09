"""Keep only detections near the predicted LOS point.

Pass-through by default: with no LOS reference set yet (context["los_point"]
is None -- no click in the viewer, or no los-*.csv found for this file), every
detection survives untouched. Once a reference exists, a detection survives
only if its box centre sits within MAX_DIST_PX of the predicted point;
everything else is assumed to be a false positive the detector picked up
elsewhere in the frame -- clutter, a sensor artifact, a second unrelated hot
spot -- since the real target is assumed to be wherever gimbal attitude alone
predicts a world-static point would be.
"""

import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

# Tunable lives in config.py -- see it for why it's set the way it is.
MAX_DIST_PX = config.LOS_PROXIMITY_MAX_DIST_PX


def filter(boxes, context):
    los_point = context.get("los_point")
    if los_point is None:
        return boxes
    lx, ly = los_point
    kept = []
    for box in boxes:
        x, y, w, h = box[:4]  # tolerate a trailing confidence score, ignore it here
        bcx, bcy = x + w / 2.0, y + h / 2.0
        if ((bcx - lx) ** 2 + (bcy - ly) ** 2) ** 0.5 <= MAX_DIST_PX:
            kept.append(box)
    return kept
