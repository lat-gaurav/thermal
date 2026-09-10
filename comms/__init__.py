"""The Cube (CubeOrangePlus) MAVLink link: live attitude down, detections up.

RPI_COMMS.md at the repo root is the protocol contract and this package
implements exactly that and nothing more. Read it before changing anything here;
in particular sections 3.1 (the az/el frame convention) and 6 (the two pymavlink
prerequisites) are the two places where a plausible-looking change silently
stops working rather than failing.

    comms.dialect     import pymavlink with the custom messages available
    comms.bearing     pixel -> camera-frame (az, el) for the uplink
    comms.cube_link   the link itself: CubeLink

WHAT THIS UNLOCKS FOR THIS REPO. The detector runs on a live feed already, but
SmoothTracker and filters/los_proximity need a per-frame camera attitude, which
until now existed only as a recorded los-*.csv beside a .rawrec. CubeLink.q_at()
is that same lookup against a live 30 Hz stream, on the same CLOCK_MONOTONIC the
frames are stamped with, so the LOS half of the pipeline can run live too.
"""
