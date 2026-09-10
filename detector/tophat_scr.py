"""White top-hat + signal-to-clutter blob detector.

A grayscale morphological opening reconstructs everything broader than SE_SIZE
(smooth gradients, the earth/sky horizon) and gets subtracted off, leaving only
things narrower than the opening -- that's the top-hat response. A candidate is
then only kept if its top-hat response is much stronger than the response in
the ring of background around it (signal-to-clutter, SCR): a real isolated
target scores enormously against quiet background, while a leftover gradient
edge or textured clutter scores little because its neighbors respond almost as
much.

The local background average must exclude the candidate's own footprint --
averaging over a box the same size as the target would fold the target's own
bright top-hat response into its "background" and self-suppress the score.
So the background here is the mean of an annulus: a big box average minus a
small (target-sized) box average, rescaled for the pixel counts.

In a very flat, quiet patch the annulus average can itself be near zero, and
then SCR blows up on ordinary sensor noise: a 5-7 DN top-hat blip divided by
a ~0.5 DN annulus still clears a MIN_SCR of 12, even though a real target's
top-hat response here runs 80-140 DN. NOISE_FLOOR guards against that -- it's
added to the annulus before dividing, so the ratio is capped by roughly
tophat/NOISE_FLOOR whenever the true background is near zero, keeping noise
below MIN_SCR while barely denting a real target's score.

COST. Profiled on a Pi 5 (OpenCV 4.10, 4 threads) over 24 real frames: the
morphological opening is 91-95% of this function and everything else together is
~25 ms. Its cost is exactly linear in pixels x SE area -- measured at 7.4e-11 s
per pixel-element, constant from SE 15 to SE 75 -- because OpenCV has no
decomposition for an ellipse and evaluates all 2337 elements per pixel. Two
consequences worth knowing before touching anything:

  * DETECTOR_SE_SIZE is a cost knob as much as a detection knob. Cost goes as
    its SQUARE: 55 -> 75 costs 425 ms instead of 227.
  * A full 1280x1024 frame cannot be searched at frame rate on this hardware,
    and no micro-optimisation changes that. What makes the live pipeline work is
    searching a tracker-sized crop instead (tools/live_track.py), where this
    runs in ~30 ms.

The implementation below is written for that cost and is BIT-EXACT against the
obvious formulation of the same maths; see the comments in detect() for why each
step is equivalent rather than approximate. Verified over 24 real frames:
identical box geometry on all of them, scores agreeing to 3e-5 (float
reassociation in the annulus term, which is better conditioned here than in the
original form).
"""
import pathlib
import sys

import cv2
import numpy as np

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

# All tunables live in config.py -- see it for why each one is set the way
# it is; nothing here should be edited without editing there.
SE_SIZE = config.DETECTOR_SE_SIZE
OUTER_SIZE = config.DETECTOR_OUTER_MULTIPLE * SE_SIZE   # outer box for the background annulus
MIN_AREA = config.DETECTOR_MIN_AREA
MAX_AREA = config.DETECTOR_MAX_AREA
MIN_SCR = config.DETECTOR_MIN_SCR
NOISE_FLOOR = config.DETECTOR_NOISE_FLOOR
INNER_AREA = SE_SIZE * SE_SIZE
OUTER_AREA = OUTER_SIZE * OUTER_SIZE


def detect(frame):
    """Return (x, y, w, h, score) boxes for candidate targets in a raw sensor
    frame. score is the candidate's peak SCR -- already computed for the
    MIN_SCR gate below, so this just keeps it instead of throwing it away.
    Consumers that fuse detections with something else (temporal tracking,
    LOS gating) can use it as a confidence weight."""
    se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (SE_SIZE, SE_SIZE))

    # White top-hat, in the frame's OWN dtype rather than float32. This is the
    # whole cost of the function, and uint8 morphology is 2.9x faster than the
    # same call on float32 (227 ms vs 668 ms on a full frame).
    #
    # It is exact, not an approximation. An opening is anti-extensive, so
    # opened <= img everywhere and img - opened can never be negative -- the
    # explicit clip this replaces was always a no-op (verified: min(img-opened)
    # is 0, not a negative number). The difference of two values in 0..255 also
    # fits in 0..255, so nothing saturates.
    tophat = cv2.morphologyEx(frame, cv2.MORPH_TOPHAT, se).astype(np.float32)

    inner_avg = cv2.blur(tophat, (SE_SIZE, SE_SIZE))
    outer_avg = cv2.blur(tophat, (OUTER_SIZE, OUTER_SIZE))

    # The annulus mean, plus NOISE_FLOOR, in one pass.
    #
    # The pixel-count rescale collapses: with OUTER_SIZE = 3 * SE_SIZE the outer
    # box holds exactly 9x the pixels of the inner one, so
    #     (outer*oa - inner*ia) / (oa - ia)  =  (9*outer - inner) / 8
    # and the areas cancel entirely. Both weights are then exact binary
    # fractions (1.125, -0.125), so this form introduces no rounding of its own
    # AND avoids scaling a float32 by ~2.7e4 the way the original did, which is
    # why the two disagree by ~3e-5 in the last digits of a score.
    w_out = OUTER_AREA / (OUTER_AREA - INNER_AREA)
    w_in = -INNER_AREA / (OUTER_AREA - INNER_AREA)
    denom = cv2.addWeighted(outer_avg, w_out, inner_avg, w_in, NOISE_FLOOR)

    # scr >= MIN_SCR without ever forming scr. The denominator is strictly
    # positive -- annulus is a mean of non-negative top-hat values and
    # NOISE_FLOOR is 1.0 -- so dividing through by it preserves the inequality:
    #     tophat / denom >= MIN_SCR   <=>   tophat >= MIN_SCR * denom
    # That trades a whole-frame division for a whole-frame multiply and drops
    # one 1.3 M-element temporary.
    mask = (tophat >= denom * MIN_SCR).astype(np.uint8)
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    boxes = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if MIN_AREA <= area <= MAX_AREA:
            # The division survives, but only inside a surviving box -- a few
            # hundred pixels rather than the whole frame.
            sub = tophat[y:y + h, x:x + w] / denom[y:y + h, x:x + w]
            boxes.append((x, y, w, h, float(sub.max())))
    return boxes
