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
OUTER_SIZE = 3 * SE_SIZE   # outer box for the background annulus
MIN_AREA = config.DETECTOR_MIN_AREA
MAX_AREA = config.DETECTOR_MAX_AREA
MIN_SCR = config.DETECTOR_MIN_SCR
NOISE_FLOOR = config.DETECTOR_NOISE_FLOOR


def detect(frame):
    """Return (x, y, w, h, score) boxes for candidate targets in a raw sensor
    frame. score is the candidate's peak SCR -- already computed for the
    MIN_SCR gate below, so this just keeps it instead of throwing it away.
    Consumers that fuse detections with something else (temporal tracking,
    LOS gating) can use it as a confidence weight."""
    img = frame.astype(np.float32)

    se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (SE_SIZE, SE_SIZE))
    opened = cv2.morphologyEx(img, cv2.MORPH_OPEN, se)
    tophat = np.clip(img - opened, 0, None)

    inner_avg = cv2.blur(tophat, (SE_SIZE, SE_SIZE))
    outer_avg = cv2.blur(tophat, (OUTER_SIZE, OUTER_SIZE))
    inner_area, outer_area = SE_SIZE * SE_SIZE, OUTER_SIZE * OUTER_SIZE
    annulus = (outer_avg * outer_area - inner_avg * inner_area) / (outer_area - inner_area)
    scr = tophat / (annulus + NOISE_FLOOR)

    mask = (scr >= MIN_SCR).astype(np.uint8)
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    boxes = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if MIN_AREA <= area <= MAX_AREA:
            peak_scr = float(scr[y:y + h, x:x + w].max())
            boxes.append((x, y, w, h, peak_scr))
    return boxes
