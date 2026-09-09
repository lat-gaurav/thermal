"""Reject shapes that look like clutter, not a real compact target.

Two independent checks, both grounded in artifacts actually observed
repeatedly across unrelated frames and files in this dataset -- not
hypothetical cases:

  1. Aspect ratio. Every confirmed real target so far has been roughly
     blob-shaped (close to square). The recurring false positives found in
     this footage were long thin slivers (e.g. 2x19px, 53x2px), which this
     rejects outright.
  2. Frame-border contact. A bright region sitting right at the sensor's
     edge (x=0 in particular) has shown up again and again across frames
     that have nothing else in common -- consistent with a fixed sensor-edge
     defect, not a real, independently-moving target. Needs frame_w/frame_h
     in context; skipped (pass-through) if the caller doesn't provide them,
     since under-filtering is better than crashing on a missing key.
"""

import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

# Tunables live in config.py -- see it for why each is set the way it is.
MAX_ASPECT_RATIO = config.CLUTTER_MAX_ASPECT_RATIO
BORDER_MARGIN_PX = config.CLUTTER_BORDER_MARGIN_PX


def filter(boxes, context):
    w = context.get("frame_w")
    h = context.get("frame_h")
    kept = []
    for box in boxes:
        x, y, bw, bh = box[:4]
        aspect = max(bw, bh) / max(min(bw, bh), 1)
        if aspect > MAX_ASPECT_RATIO:
            continue
        if w is not None and h is not None:
            if x <= BORDER_MARGIN_PX or y <= BORDER_MARGIN_PX:
                continue
            if x + bw >= w - BORDER_MARGIN_PX or y + bh >= h - BORDER_MARGIN_PX:
                continue
        kept.append(box)
    return kept
