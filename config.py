"""Every tunable/changeable parameter for the detector/filter/tracker
pipeline, in one place. Every module that has a knob worth adjusting reads
it from here instead of defining its own constant.

This exists because the same idea kept ending up as two independently
tunable copies in two different files -- INITIAL_LATENCY_S in
experiment/los_static_track.py and LOS_LATENCY_S in tools/rawrec_viewer.py
were the same parameter under two names, and nothing stopped them drifting
apart. One file to tune, instead of hunting across detector/, filters/,
initialisation/, experiment/ and tools/ for the same kind of number.

Plain module, no framework: every consumer just does `import config` and
reads config.SOME_NAME. Values are grouped below by which file actually
uses them; each group's comments carry the same reasoning that used to sit
next to the constant at its original definition site.
"""

# --- detector/tophat_scr.py -----------------------------------------------
# White top-hat + signal-to-clutter blob detector.
DETECTOR_SE_SIZE = 55          # opening kernel, in px -- bigger than the
                                # target so the opening also erases it,
                                # leaving one coherent top-hat blob
DETECTOR_MIN_AREA = 9          # px, rejects single-pixel sensor noise
DETECTOR_MAX_AREA = 3000       # px, rejects anything implausibly large
DETECTOR_MIN_SCR = 12.0        # top-hat response over local clutter, the
                                # isolation test
DETECTOR_NOISE_FLOOR = 1.0     # DN, floor under the annulus so SCR can't
                                # blow up on noise where the local
                                # background is near zero

# --- filters/los_proximity.py ---------------------------------------------
# Keeps only detections near the predicted LOS point.
LOS_PROXIMITY_MAX_DIST_PX = 150.0  # how far a detection may sit from the LOS
                                    # prediction and still be kept

# --- filters/clutter_reject.py ---------------------------------------------
# Rejects shapes that look like clutter, not a real compact target.
CLUTTER_MAX_ASPECT_RATIO = 4.0  # long/short side ratio beyond this reads as
                                # a streak, not a blob
CLUTTER_BORDER_MARGIN_PX = 2    # how close to the frame edge counts as
                                # "touching" it

# --- initialisation/rightmost_isolated.py -----------------------------------
# No-click LOS init: rightmost detection isolated from all others.
INIT_MIN_ISOLATION_PX = 100.0  # a candidate counts as isolated if its
                                # nearest neighbour among the frame's other
                                # detections (if any) is at least this far away

# --- experiment/los_static_track.py: attitude timing & mount extrinsics -----
LOS_LATENCY_S = -0.070   # world-event -> attitude-lookup shift, applied on
                         # top of the los-*.csv's own qw..qz. Starting point
                         # only -- tune live with '[' / ']' in the experiment
                         # viewer while watching a static reference track.
LOS_LATENCY_STEP_S = 0.010  # per key press while live-tuning
LOS_MIN_LATENCY_S = -0.5
LOS_MAX_LATENCY_S = 0.5

# Measured camera-to-body extrinsic rotation (R_bc): rotates a vector in the
# SENSOR frame (X=boresight, Y=image-right, Z=image-down) into BODY FRD
# (X=forward, Y=right, Z=down) -- the quaternion's own convention. Roll (90,
# validated by correlating predicted vs. observed image motion over 524
# frame pairs: +90 gave corr +0.735, -90 gave -0.735) and pitch (25 up)
# TOGETHER with the axis reorder from (right, down, forward) to (forward,
# right, down) -- not a pure roll composed with a pure pitch in image-axis
# order. Taken from a companion rig's extrinsics.json; not re-derived or
# re-verified against this specific airframe.
MOUNT_R_BC = [
    [0.906307787037, -0.422618261741, 0.0],
    [0.0, 0.0, 1.0],
    [-0.422618261741, -0.906307787037, 0.0],
]

# --- experiment/los_static_track.py: SmoothTracker --------------------------
TRACKER_BASE_GATE_PX = 50.0        # gate radius with a fresh fix and no rotation
TRACKER_GATE_PX_PER_FRAME = 10.0   # growth per frame since the last accepted fix
TRACKER_GATE_PX_PER_DEG_S = 1.5    # growth per deg/s of angular rate
TRACKER_MAX_GATE_PX = 350.0
TRACKER_OMEGA_REF_DEG_S = 100.0    # detector confidence is halved around this rate
TRACKER_CONF_REF = 50.0            # an SCR at or above this counts as "fully
                                   # confident" for candidate selection --
                                   # caps confidence's pull so a
                                   # strong-but-farther detection can't
                                   # automatically beat a weak-but-closer one
TRACKER_ALPHA_MAX = 0.7            # hard cap on how far one frame can pull
                                   # toward a measurement -- this is what
                                   # forbids a visible jump
TRACKER_MISS_GAIN = 0.15           # extra pull allowed per frame missed in a row
TRACKER_CROP_MARGIN_PX = 200.0     # added beyond the tracker's own gate radius
                                   # when sizing a search crop, so a
                                   # detector's own morphological kernel
                                   # doesn't react to the crop's hard edge

# --- tools/rawrec_viewer.py & tools/web_viewer.py: shared UI ----------------
VIEWER_HUD_H = 30       # height in px of the info strip drawn above the frame
VIEWER_BOX_COLOR = (0, 200, 255)
VIEWER_SPEED_STEP = 1.5
VIEWER_MIN_SPEED = 1.0 / 16
VIEWER_MAX_SPEED = 16.0
VIEWER_LOS_TRACK_COLOR = (0, 255, 0)     # tracker fused a detection this frame
VIEWER_LOS_COAST_COLOR = (0, 165, 255)   # pure attitude dead-reckoning, no fix
