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

MAX_DIST_PX = 150  # how far a detection may sit from the LOS prediction and
                  # still be kept. Roughly the largest confirmed target
                  # footprint seen so far in this dataset (~45px), plus slack
                  # for reprojection error (unverified mount/latency figures,
                  # SLERP/timestamp join) and the target's own real motion
                  # away from the static-world assumption.


def filter(boxes, context):
    los_point = context.get("los_point")
    if los_point is None:
        return boxes
    lx, ly = los_point
    kept = []
    for (x, y, w, h) in boxes:
        bcx, bcy = x + w / 2.0, y + h / 2.0
        if ((bcx - lx) ** 2 + (bcy - ly) ** 2) ** 0.5 <= MAX_DIST_PX:
            kept.append((x, y, w, h))
    return kept
