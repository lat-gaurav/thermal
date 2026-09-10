"""Pixel -> the final line of sight, with the camera's mounting applied.

    solver = LosSolver(focal_px=1516)
    e_sensor = solver.sensor_vector(u, v)     # the camera's own frame
    e_body   = solver.body(u, v)              # airframe FRD, mount applied
    e_ned    = solver.ned(u, v, q)            # world NED, attitude applied

THREE FRAMES, AND THE TWO ROTATIONS BETWEEN THEM

    sensor    X = boresight, Y = image-right, Z = image-down
      |  R_bc   -- the camera's MOUNTING. Fixed, measured, config.MOUNT_R_BC.
      v
    body FRD  X = forward, Y = right, Z = down
      |  q     -- the vehicle's ATTITUDE. Live, from the Cube at 30 Hz.
      v
    NED       X = north, Y = east, Z = down

WHY THIS IS NOT WHAT comms/bearing.py DOES, and why both exist. bearing.py
produces the `bearing_az`/`bearing_el` the Cube actually consumes, and for that
it must apply ONLY the mount roll -- the firmware applies the mount pitch itself
from LAT_CAM_PITCH, so applying it here too would double it. This module is the
opposite: it applies the mount in FULL, because a world-frame LOS has no second
party to finish the job. The two are not redundant and neither can be derived
from the other without knowing which convention the consumer expects.

R_bc CARRIES BOTH ANGLES AND THE AXIS REORDER, and is not a roll composed with a
pitch in image-axis order -- see config.MOUNT_R_BC's comment. Roll +90 was
measured by correlating predicted against phase-correlated observed image motion
over 524 frame pairs (+90 gives +0.735, -90 gives -0.735: an exact negation,
i.e. 180 degrees about the boresight). Pitch is 25 degrees up.

WHAT THE FIRMWARE DOES WITH THE NED LOS TODAY: NOTHING. RPI_COMMS.md section 3
is explicit that `los_n/los_e/los_d` are decoded and ignored, because the
`frame=1` path is not wired up, and that `frame` must stay 0. So a NED LOS on
the wire is for the record and for whenever that path is finished -- guidance
still steers on bearing_az/bearing_el. Sending it costs nothing: those fields
are already in the 52-byte message, zeroed.
"""
import math
import pathlib
import sys

import numpy as np

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config


def quat_to_matrix(q):
    """Hamilton (w, x, y, z) -> rotation matrix, applied FORWARD as body -> NED.

    ArduPilot's own header comments claim the opposite direction; the code does
    not, and RPI_COMMS.md section 4.1 carries the citation. Used forward, this is
    what getRotationBodyToNED() returns.
    """
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class LosSolver:
    """Pixel and attitude -> LOS in the sensor, body and NED frames."""

    def __init__(self, focal_px=None, cx=None, cy=None, width=1280, height=1024,
                 r_bc=None):
        self.f = float(config.CAM_FOCAL_PX if focal_px is None else focal_px)
        self.cx = float(width / 2.0 if cx is None else cx)
        self.cy = float(height / 2.0 if cy is None else cy)
        self.R_bc = np.array(config.MOUNT_R_BC if r_bc is None else r_bc, dtype=float)

    def sensor_vector(self, u, v):
        """Unit ray in the camera's OWN frame: no mount correction of any kind."""
        e = np.array([self.f, float(u) - self.cx, float(v) - self.cy], dtype=float)
        n = np.linalg.norm(e)
        return e / n if n > 1e-12 else np.array([1.0, 0.0, 0.0])

    def body(self, u, v):
        """Unit LOS in airframe FRD: the camera's mounting fully applied."""
        return self.R_bc @ self.sensor_vector(u, v)

    def ned(self, u, v, q):
        """Unit LOS in world NED: mounting, then the vehicle's attitude."""
        return quat_to_matrix(q) @ self.body(u, v)

    def body_az_el(self, u, v):
        """(az, el) radians in the BODY frame, for diagnostics.

        NOT what goes on the wire -- the wire wants camera-frame az/el from
        comms/bearing.py, with the pitch left for the Cube. This is the same
        angles measured from the airframe's nose instead, which is what a human
        reading a flight log wants to see.
        """
        X, Y, Z = self.body(u, v)
        return math.atan2(Y, X), math.atan2(-Z, math.hypot(X, Y))

    def is_conformal(self, tol=1e-6):
        """Is R_bc a proper rotation? A mount matrix that is not costs you a
        silently skewed LOS rather than an error, so this is worth asserting
        once at startup rather than trusting the file it came from."""
        should_be_i = self.R_bc.T @ self.R_bc
        return (np.allclose(should_be_i, np.eye(3), atol=tol)
                and abs(np.linalg.det(self.R_bc) - 1.0) < tol)
