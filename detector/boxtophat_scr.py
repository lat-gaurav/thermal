"""Box top-hat + signal-to-clutter blob detector: tophat_scr's test, made affordable.

Same three-part decision as detector/tophat_scr.py -- reconstruct the smooth
background by a morphological opening, subtract it, and keep only candidates
whose top-hat response towers over the response in the ring around them (SCR).
Read that file first; every constant here is the same constant, from the same
place in config.py, chosen for the same reasons.

WHY THIS EXISTS. tophat_scr costs 247 ms on a full 1280x1024 frame on this Pi 5,
of which 92% is one call: the opening with a 55x55 ELLIPSE. OpenCV has no
decomposition for an ellipse, so it evaluates all 2337 elements at every pixel,
and the cost is linear in pixels x SE area. No arrangement of the surrounding
arithmetic changes that. A full-frame search at 25 fps is therefore impossible
with an ellipse, and full-frame search is what acquisition needs.

This detector runs the same test in 27 ms -- 37 fps mean, 33 fps on the worst of
24 real frames -- through three changes. The first is a genuine behaviour change
and the other two are exact.

1. A RECTANGULAR STRUCTURING ELEMENT, which OpenCV runs SEPARABLY: two 1-D
   passes instead of 2337 taps, 7 ms instead of 227.

   This is the one thing that makes it a different detector, so be clear about
   what changes. The ellipse is contained in the square, and an opening with a
   larger element removes more, so

       tophat_rect >= tophat_ellipse   at every pixel

   (verified on real frames: min(tophat_rect - tophat_ellipse) = 0). The square
   top-hat can therefore never fail to respond where the round one responds --
   it is uniformly MORE sensitive, and is then re-gated by the same SCR test.
   What it does not preserve is which pixels clear that gate, because a larger
   top-hat also raises the annulus it is divided by. Measured against
   tophat_scr over 24 real frames, 508 detections: 70% of them reappear within
   4 px, 82% within 25 px, and this detector emits 507 where tophat_scr emits
   508. So the difference is detections shifting and merging, not sensitivity
   being lost.

   The square is also anisotropic where the disc is not: it erases structures
   broader than 55 px along the image axes more readily than along a diagonal.
   For a compact point target that is immaterial. For elongated diagonal
   clutter it is not, and filters/clutter_reject.py is what handles that.

2. INTEGER BOX SUMS for the annulus, instead of float32 means. cv2.boxFilter
   with CV_32S accumulates the top-hat exactly -- a top-hat of a uint8 frame is
   uint8, and sums over a 165x165 box reach 6.9 M, well inside int32. The old
   float32 blurs rounded; these do not. Also cheaper: 2.1 ms and 2.3 ms against
   4.1 and 5.3.

3. NO PYTHON LOOP OVER COMPONENTS. The per-candidate peak SCR is taken with one
   vectorised np.maximum.at over the masked pixels, so the cost does not grow
   with how cluttered the frame is. That is what holds the worst case at 30 ms:
   the earlier loop-based version measured 35 ms mean but 46 ms on a busy frame,
   which is 21.7 fps and fails the requirement on exactly the frames that
   matter.

NOT YET VALIDATED AGAINST GROUND TRUTH. tophat_scr's thresholds were set against
observed footage, and thermal_detect.py's against 659 hand-labelled targets. The
70%/82% agreement above is agreement with tophat_scr on 24 bench frames of
clutter with no drone in them, which says nothing about whether a real target
survives. Before flying this, re-run the labelled evaluation
(thermal_detect.py --eval) on footage that contains one, and compare recall and
false-alarm rate against tophat_scr on the same frames.
"""
import pathlib
import sys

import cv2
import numpy as np

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

# Every tunable is shared with detector/tophat_scr.py, deliberately: these are
# two implementations of one test, and a threshold that drifted between them
# would make their results incomparable, which is the whole point of keeping
# both. config.py is the only place to change any of it.
SE_SIZE = config.DETECTOR_SE_SIZE
OUTER_SIZE = config.DETECTOR_OUTER_MULTIPLE * SE_SIZE
MIN_AREA = config.DETECTOR_MIN_AREA
MAX_AREA = config.DETECTOR_MAX_AREA
MIN_SCR = config.DETECTOR_MIN_SCR
NOISE_FLOOR = config.DETECTOR_NOISE_FLOOR

INNER_AREA = SE_SIZE * SE_SIZE
OUTER_AREA = OUTER_SIZE * OUTER_SIZE
ANNULUS_AREA = OUTER_AREA - INNER_AREA          # pixels in the background ring

# MORPH_RECT, not getStructuringElement(MORPH_RECT, ...): a plain ones() array
# is the same element and makes it obvious at the call site that this is the
# separable case. This is the whole reason the detector is affordable.
_SE = np.ones((SE_SIZE, SE_SIZE), np.uint8)
_INV_ANNULUS = np.float32(1.0 / ANNULUS_AREA)
_NOISE_FLOOR32 = np.float32(NOISE_FLOOR)


def detect(frame):
    """Return (x, y, w, h, score) boxes for candidate targets in a raw sensor
    frame. score is the candidate's peak SCR, the same quantity
    detector/tophat_scr.py reports, so a consumer that weights detections by
    confidence (SmoothTracker) needs no recalibration to switch between them."""
    # White top-hat in the frame's own dtype. Exact: an opening is
    # anti-extensive, so frame - opened is never negative and cannot saturate.
    tophat = cv2.morphologyEx(frame, cv2.MORPH_TOPHAT, _SE)

    # Exact integer sums over the inner (target-sized) and outer boxes. The
    # annulus mean is (outer_sum - inner_sum) / annulus_area -- the target's own
    # footprint is excluded, which is what stops a bright target from inflating
    # its own "background" and suppressing its score.
    inner_sum = cv2.boxFilter(tophat, cv2.CV_32S, (SE_SIZE, SE_SIZE), normalize=False)
    outer_sum = cv2.boxFilter(tophat, cv2.CV_32S, (OUTER_SIZE, OUTER_SIZE), normalize=False)
    np.subtract(outer_sum, inner_sum, out=outer_sum)     # in place: no third array

    denom = outer_sum.astype(np.float32)
    denom *= _INV_ANNULUS
    denom += _NOISE_FLOOR32                              # annulus mean + NOISE_FLOOR

    # scr >= MIN_SCR without forming scr: the denominator is strictly positive
    # (a mean of non-negative top-hat values, plus a positive floor), so
    # multiplying through preserves the inequality.
    tophat_f = tophat.astype(np.float32)
    mask = (tophat_f >= denom * MIN_SCR).astype(np.uint8)

    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return []

    # Peak SCR per component, vectorised over the thresholded pixels only. The
    # division happens here and only here, on the sparse set that passed.
    sel = mask.view(bool)
    peak = np.zeros(n, np.float32)
    np.maximum.at(peak, labels[sel], tophat_f[sel] / denom[sel])

    areas = stats[1:, 4]
    keep = np.nonzero((areas >= MIN_AREA) & (areas <= MAX_AREA))[0] + 1
    return [(int(stats[i, 0]), int(stats[i, 1]), int(stats[i, 2]), int(stats[i, 3]),
             float(peak[i])) for i in keep]
