"""Pixel -> camera-frame (az, el) for DETECTION_TARGET_DATA.

THE ONE RULE (RPI_COMMS.md section 3.1): send the bearing in the CAMERA's own
frame and let the Cube apply the mount correction. If both sides correct, it is
applied twice.

...with one exception the protocol document does not cover, and it matters:
the firmware models mount PITCH only. Mount ROLL is not in its model at all, so
the roll has to come off HERE or nothing downstream ever removes it. This camera
is bolted on rolled 90 deg about its boresight, which is not a small correction
-- it exchanges the two image axes. So:

    ROLL   removed here, from config.CAM_ROLL_DEG
    PITCH  never touched here; the Cube applies LAT_CAM_PITCH

The roll sign is a physical fact about how the core was bolted on and cannot be
derived from software. config.CAM_ROLL_DEG carries the measurement it came from
(524 frame pairs, +90 vs -90 giving an exact negation of the correlation).

Frames, in the order they are traversed:

    image/vision   x right, y down, z forward     (OpenCV pixel convention)
    camera FRD     X boresight, Y right, Z down   (what az/el describe)

    e_cam = (cos el * cos az,  cos el * sin az,  -sin el)
    az = atan2(Y, X)      az > 0 = target right of boresight
    el = atan2(-Z, hypot(X, Y))    el > 0 = target above boresight
"""
import math
import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config


class CamModel:
    """Pixel -> bearing for one camera geometry.

    focal_px and the principal point are in OUTPUT pixels, i.e. the 1280x1024
    stream, not the 640x512 sensor behind it. The principal point defaults to
    the exact image centre and has never been calibrated -- along with the total
    absence of any distortion model, that is the dominant residual in these
    bearings, worth a couple of degrees at the field edge on LWIR glass.
    """

    def __init__(self, focal_px=None, cx=None, cy=None, width=1280, height=1024,
                 roll_deg=None):
        self.f = float(config.CAM_FOCAL_PX if focal_px is None else focal_px)
        self.cx = float(width / 2.0 if cx is None else cx)
        self.cy = float(height / 2.0 if cy is None else cy)
        self.roll_deg = float(config.CAM_ROLL_DEG if roll_deg is None else roll_deg)
        r = math.radians(self.roll_deg)
        self._cr, self._sr = math.cos(r), math.sin(r)

    def vector(self, u, v):
        """Pixel -> unit LOS in camera FRD, mount roll removed, pitch untouched."""
        x = float(u) - self.cx          # image right
        y = float(v) - self.cy          # image down
        z = self.f                      # image forward
        # De-roll about the optical axis: rotate the ray by -roll to undo the mount.
        xr = x * self._cr + y * self._sr
        yr = -x * self._sr + y * self._cr
        X, Y, Z = z, xr, yr             # vision (x, y, z) -> FRD (X=z, Y=x, Z=y)
        n = math.sqrt(X * X + Y * Y + Z * Z)
        if n < 1e-12:
            return 1.0, 0.0, 0.0
        return X / n, Y / n, Z / n

    def az_el(self, u, v):
        """Pixel -> (az, el) in radians, ready for the wire."""
        X, Y, Z = self.vector(u, v)
        return math.atan2(Y, X), math.atan2(-Z, math.hypot(X, Y))

    def pixel_of(self, az, el):
        """(az, el) radians in the CAMERA frame -> pixel (u, v), or None if behind.

        The exact inverse of az_el(), including the mount roll, so a bearing that
        came off the wire can be put back where it belongs in the image. That is
        what lets the radar cue be compared against a detection in pixels, which
        is the only space where "nearest" means anything to a detector.

        Returns None when the direction is at or behind the image plane (X <= 0):
        a cue can legitimately point behind the aircraft, and projecting that
        would silently produce a mirrored pixel somewhere in frame.
        """
        ce = math.cos(el)
        X, Y, Z = ce * math.cos(az), ce * math.sin(az), -math.sin(el)
        if X <= 1e-9:
            return None
        # Scale the ray to the image plane at X = f, undoing vector()'s (X=z, Y=x, Z=y)
        t = self.f / X
        xr, yr = Y * t, Z * t
        # Re-roll: vector() applied R(-roll) to (x, y), so invert with R(+roll).
        x = xr * self._cr - yr * self._sr
        y = xr * self._sr + yr * self._cr
        return x + self.cx, y + self.cy

    def deg_per_px(self):
        """Angular scale at the image centre, for sanity-checking a bearing by eye."""
        return math.degrees(math.atan(1.0 / self.f))
