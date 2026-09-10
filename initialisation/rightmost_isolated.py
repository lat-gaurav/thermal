"""Pick an initial LOS point from raw detector boxes with no click.

Rule: the RIGHTMOST candidate that is spatially isolated from every other
candidate in the same frame (nearest-neighbour centre distance at least
MIN_ISOLATION_PX, or simply the only candidate present). This camera's raw
frame has sky on the right and ground on the left (see the
camera-mount-orientation note); a drone still far out in the sky is expected
to show up as a lone, uncluttered detection out there, not part of a clump
of ground clutter -- so instead of a click, the user just steps to a frame
that visibly has such a detection and asks for it.
"""

import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

# Tunable lives in config.py -- see it for why it's set the way it is.
MIN_ISOLATION_PX = config.INIT_MIN_ISOLATION_PX


def init(boxes, context=None, min_isolation_px=MIN_ISOLATION_PX):
    """Return (x, y), or None if no candidate qualifies.

    `context` is accepted and ignored: every initialiser takes the same
    (boxes, context) shape as filters/ do, so a caller does not have to know
    which one is loaded. This one needs nothing but the boxes.
    """
    if not boxes:
        return None
    centers = [(x + w / 2.0, y + h / 2.0) for x, y, w, h, *_ in boxes]
    isolated = []
    for i, c in enumerate(centers):
        others = centers[:i] + centers[i + 1:]
        if not others:
            isolated.append(c)
            continue
        nearest = min(((c[0] - o[0]) ** 2 + (c[1] - o[1]) ** 2) ** 0.5 for o in others)
        if nearest >= min_isolation_px:
            isolated.append(c)
    if not isolated:
        return None
    return max(isolated, key=lambda c: c[0])  # rightmost = closest to the sky side
