"""Single-object tracker: OpenCV's CSRT (discriminative correlation filter).

Rule: given one bounding box on one frame, follow that specific patch of
pixels forward by appearance alone -- no detector, no attitude, nothing else
in this repo is consulted. This is the "point at a thing on screen and keep
following it" tool: useful for a target the detector/LOS pipeline doesn't
pick up, or as a second, independent check on where the LOS reprojection
says the target actually went.

CSRT over KCF/MIL: slower per frame, but holds on through partial occlusion
and scale change far better, which matters more here than raw speed -- the
callers of this module replay logged frames, they are not real-time-bound.

INTERFACE. create() returns an object exposing:
    init(frame_bgr, bbox)          bbox = (x, y, w, h) in pixels, floats
    update(frame_bgr) -> (ok, bbox)
which is exactly cv2's own Tracker API -- this wrapper exists so a caller
never has to know which OpenCV tracker class backs it, and so a different
backend can be dropped into sot/ later without touching any caller.
"""
import cv2


class _CSRTTracker:
    def __init__(self):
        self._t = cv2.TrackerCSRT_create()

    def init(self, frame_bgr, bbox):
        # cv2's CSRT wants integer pixels; callers may reasonably hand in
        # floats (e.g. straight from a mouse-drag rectangle).
        x, y, bw, bh = bbox
        self._t.init(frame_bgr, (int(x), int(y), int(bw), int(bh)))

    def update(self, frame_bgr):
        ok, bbox = self._t.update(frame_bgr)
        return ok, bbox


def create():
    return _CSRTTracker()
