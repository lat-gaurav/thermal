"""Import pymavlink such that this airframe's custom messages actually exist.

    from comms.dialect import mavutil, MAVLINK_MSG_ID_GCS_TARGET_BEARING

Two environment variables have to be set BEFORE pymavlink is first imported, and
both failures are the quiet kind (RPI_COMMS.md section 6):

  MAVLINK20=1        without it pymavlink binds its v1.0 dialect, whose message-id
                     field is 8 bits and physically cannot represent 42050/42051.
  MAVLINK_DIALECT    selects the regenerated dialect that has our two messages.
                     Stock ardupilotmega does not: sending raises AttributeError
                     on the first detection, and an incoming GCS_TARGET_BEARING
                     is dropped SILENTLY as an unknown id, which is
                     indistinguishable from firmware that does not send it.

Importing this module is the only supported way to get at mavutil in this repo,
because "set two env vars first" is not a thing that can be enforced after the
fact -- once pymavlink is imported, changing them does nothing.
"""
import os
import sys

DIALECT = "thermal_link"

MAVLINK_MSG_ID_DETECTION_TARGET_DATA = 42050
MAVLINK_MSG_ID_GCS_TARGET_BEARING = 42051
MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31

if "pymavlink" in sys.modules:  # pragma: no cover - a caller's ordering mistake
    raise ImportError(
        "pymavlink was imported before comms.dialect, so MAVLINK20/MAVLINK_DIALECT "
        "came too late to take effect. Import comms.dialect first.")

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", DIALECT)

from pymavlink import mavutil  # noqa: E402  (must follow the env vars)

MAV = mavutil.mavlink


def check(verbose=False):
    """(ok, detail) -- is the dialect in use one that carries both messages?

    Checked against the bound mavlink module rather than by importing the
    dialect by name, so this reports what THIS process will actually put on the
    wire, including the case where something imported pymavlink first and got
    stock ardupilotmega instead.
    """
    missing = []
    for name, mid in (("detection_target_data", MAVLINK_MSG_ID_DETECTION_TARGET_DATA),
                      ("gcs_target_bearing", MAVLINK_MSG_ID_GCS_TARGET_BEARING)):
        cls = getattr(MAV, "MAVLink_%s_message" % name, None)
        if cls is None:
            missing.append("%s (%d) absent" % (name.upper(), mid))
        elif cls.id != mid:
            missing.append("%s has id %d, expected %d" % (name.upper(), cls.id, mid))
    if missing:
        return False, ("%s; regenerate with:  python3 comms/gen_dialect.py"
                       % "; ".join(missing))
    detail = "dialect=%s wire=MAVLink%s 42050+42051 present" % (
        os.environ.get("MAVLINK_DIALECT"), "2" if os.environ.get("MAVLINK20") == "1" else "1")
    if verbose:
        print("[dialect] " + detail)
    return True, detail
