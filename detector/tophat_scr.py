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
import cv2
import numpy as np

SE_SIZE = 55         # opening kernel, in px -- bigger than the target so the
                      # opening also erases it, leaving one coherent top-hat blob
OUTER_SIZE = 3 * SE_SIZE   # outer box for the background annulus
MIN_AREA = 9         # px, rejects single-pixel sensor noise
MAX_AREA = 3000        # px, rejects anything implausibly large
MIN_SCR = 12.0        # top-hat response over local clutter, the isolation test
NOISE_FLOOR = 1.0     # DN, floor under the annulus so SCR can't blow up on
                      # noise where the local background is near zero


def detect(frame):
    """Return (x, y, w, h) boxes for candidate targets in a raw sensor frame."""
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
            boxes.append((x, y, w, h))
    return boxes
