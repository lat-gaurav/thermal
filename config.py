"""Every configurable parameter in this repository, in one file.

Every module that has a knob worth turning reads it from here instead of
defining its own constant. Plain module, no framework: `import config` and read
`config.SOME_NAME`.

This exists because the same idea kept ending up as two independently tunable
copies in two different files -- INITIAL_LATENCY_S in
experiment/los_static_track.py and LOS_LATENCY_S in tools/rawrec_viewer.py were
the same parameter under two names, and nothing stopped them drifting apart.

WHAT IS DELIBERATELY *NOT* HERE, because it is not configurable:

  * Wire formats. The .rawrec magic numbers, header length and 32-byte record
    layout live in flight/rawrec.py; MAVLink message ids live in
    comms/dialect.py. Changing any of them does not tune anything, it breaks
    compatibility with files and firmware that already exist.
  * Measured physical constants that are not choices: the annulus rescale in
    detector/, the quaternion conventions, R_bc's derivation. MOUNT_R_BC is here
    because it is per-airframe data, not because it is a preference.
  * Anything systemd owns. Which mode the service runs, where it writes, whether
    it records or transmits -- those are in /etc/default/thermal-live, because
    they are per-deployment and must be changeable without touching the repo.
    deploy/thermal-live.env is the annotated reference copy.

CONVENTIONS. Units are in the name: _S seconds, _MS milliseconds, _US
microseconds, _PX pixels, _DEG degrees, _MB megabytes, _HZ hertz. Where a value
was measured rather than chosen, the comment says what measured it, because the
next person to change it needs to know whether they are overriding an opinion or
an observation.
"""

# ===========================================================================
# 1. THE CAMERA
# ===========================================================================
# The Artosyn "Sirius" thermal core. flight/camera.py, tools/*.
CAMERA_DEVICE = "/dev/thermal0"   # udev symlink pinned by USB serial
                                  # (deploy/99-thermal-core.rules). NEVER a
                                  # /dev/videoN name: those are assigned in
                                  # enumeration order and do move -- one
                                  # debugging session saw 4 -> 6 -> 9 -> 11 ->
                                  # 4 -> 24 (docs/DEVICE_NODES.md). /dev/video1
                                  # is the UVC METADATA node: it opens without
                                  # error and never yields a frame.
CAMERA_WIDTH = 1280              # the core advertises exactly one size
CAMERA_HEIGHT = 1024
CAMERA_FOURCC = "GREY"           # cheapest of the four it offers at 1 byte/px.
                                 # The sensor is really 640x512 -- three
                                 # quarters of these pixels are interpolated by
                                 # the core's own ISP.
CAMERA_FPS = 25                  # 25 or 50. The core forgets this on every
                                 # power cycle, so it is re-applied per start.
                                 # 25 is the only rate the USB path honours end
                                 # to end: mode 50 measures 30.17 fps, not 50,
                                 # because 1280x1024 at 50 fps is 524 Mbps
                                 # against a 480 Mbps link.
CAMERA_FIRST_FRAME_TIMEOUT_S = 5.0   # give up if an opened device sends nothing
CAMERA_THREAD_JOIN_S = 2.0           # the grab thread parks in a blocking
                                     # read(); tearing down around it aborts the
                                     # process with "exception not rethrown"

# ===========================================================================
# 2. DETECTORS
# ===========================================================================
# --- detector/tophat_scr.py and detector/boxtophat_scr.py ------------------
# Two implementations of ONE test -- white top-hat plus signal-to-clutter --
# sharing these constants on purpose. A threshold that drifted between them
# would make their results incomparable, which is the whole reason both exist.
DETECTOR_SE_SIZE = 55          # opening kernel, in px -- bigger than the
                               # target so the opening also erases it, leaving
                               # one coherent top-hat blob.
                               # ALSO THE COST KNOB, quadratically: the opening
                               # is ~92% of detect() and its cost goes as
                               # pixels x SE area (measured 7.4e-11 s per
                               # pixel-element on a Pi 5, flat from SE 15 to 75,
                               # because OpenCV has no decomposition for an
                               # ellipse and evaluates every element at every
                               # pixel). Whole detect() on a 520x300 tracker
                               # crop: SE 35 -> 12 ms, 55 -> 30 ms, 75 -> 56 ms,
                               # against a 40 ms budget at 25 fps. Re-measure
                               # with tools/tophat_profile.py --se-scan.
DETECTOR_OUTER_MULTIPLE = 3    # the background annulus is this many times
                               # SE_SIZE across. 3 is load-bearing arithmetic,
                               # not a taste: it makes the outer box exactly 9x
                               # the inner one, so the pixel-count rescale
                               # collapses to (9*outer - inner)/8 with weights
                               # that are exact binary fractions. Change it and
                               # both detectors still work, but they stop being
                               # free of rounding.
DETECTOR_MIN_AREA = 9          # px, rejects single-pixel sensor noise
DETECTOR_MAX_AREA = 3000       # px, rejects anything implausibly large
DETECTOR_MIN_SCR = 14.0        # top-hat response over local clutter: the
                               # isolation test, and the detection gate
DETECTOR_NOISE_FLOOR = 1.0     # DN, floor under the annulus so SCR cannot blow
                               # up on noise where the local background is near
                               # zero. In a flat patch a 5-7 DN blip over a
                               # ~0.5 DN annulus would otherwise clear MIN_SCR,
                               # where a real target's response runs 80-140 DN.

# --- thermal_detect.py ------------------------------------------------------
# A different detector: local contrast, an isolation "moat", and a crowd count.
# THESE NUMBERS ARE THE MOST EXPENSIVE IN THIS FILE. They were tuned on a
# training half of 659 hand-labelled targets and then scored ONCE on a held-out
# half: recall 97.0% at 0.065 false alarms/frame. Measured against real flight
# footage with no drone airborne it emits 0.33 FA/frame, where both top-hat
# detectors emit ~13. Do not adjust them without re-running the labelled
# evaluation -- thermal_detect.py --eval.
TD_MED_W = 31              # local-background median window, px
TD_RING_IN = 3             # the moat annulus, px. The moat is I minus the
TD_RING_OUT = 7            # BRIGHTEST pixel in this ring -- a maximum, not a
                           # mean, because a mean is diluted by the quiet parts
                           # of the ring, which is exactly how a peak with
                           # clutter on one side only scores as isolated.
TD_NMS_W = 5               # a candidate must be the maximum of this box
TD_CAND_FLOOR = 6          # contrast below which a peak is not even a
                           # candidate. This sets the population the crowd count
                           # is measured against, so it is not independent of
                           # TD_MAX_CROWD: raising one without the other
                           # silently rescales the other's meaning.
TD_CROWD_W = 101           # box within which other candidates are counted
TD_MERGE_PX = 6            # detections closer than this are one object
TD_MIN_CONTRAST = 50.0     # I minus the local median
TD_MIN_MOAT = 2.0          # I minus the ring maximum
TD_MAX_CROWD = 20          # targets sit among ~3 other peaks; noise among ~119

# ===========================================================================
# 3. FILTERS   (filters/, run on the detector's output every frame)
# ===========================================================================
# --- filters/clutter_reject.py ---------------------------------------------
CLUTTER_MAX_ASPECT_RATIO = 4.0  # long/short side ratio beyond this reads as a
                                # streak, not a blob. Grounded in artifacts
                                # observed repeatedly across unrelated frames --
                                # 2x19 px and 53x2 px slivers -- not in theory.
CLUTTER_BORDER_MARGIN_PX = 2    # how close to the frame edge counts as
                                # "touching" it. A bright region at x=0 has
                                # recurred across frames with nothing else in
                                # common, consistent with a fixed sensor-edge
                                # defect rather than a real moving target.

# --- filters/los_proximity.py ----------------------------------------------
LOS_PROXIMITY_MAX_DIST_PX = 150.0  # how far a detection may sit from the LOS
                                   # prediction and still be kept. Pass-through
                                   # when no LOS reference exists yet.

# ===========================================================================
# 4. ACQUISITION   (initialisation/, decides WHAT to start tracking)
# ===========================================================================
# Chosen BY NAME, never by whichever file sorts first: dropping a new .py into
# initialisation/ used to change what acquires the target, and in flight that
# must not depend on a filename.
FLIGHT_INITIALISER = "cue_nearest"         # tools/flight_pipeline.py
VIEWER_INITIALISER = "rightmost_isolated"  # the offline viewers, which have no
                                           # cue, so the cue-based one could
                                           # never return anything there.
# CUELESS FALLBACK. When no cue is arriving at all, cue_nearest can never
# return anything and the pipeline sends valid=0 forever -- a sortie with a
# dead radar link produces no usable detection uplink whatsoever. Set this to
# an initialisation/ module name to acquire with that one instead, for as long
# as the cue is absent; None restores the cue-only behaviour.
#
# KNOW WHAT THIS COSTS. rightmost_isolated is the strategy flight deliberately
# moved away from: against ~13 false alarms per frame it locks onto clutter,
# and a bench scene pointed at a wall produced a confident lock reporting
# valid=1 on 840 of 841 frames. Worse, with no cue there is also no release
# authority (CUE_DROP_DEG below is only evaluated while the cue is valid), so
# nothing can drop that lock until the cue comes back. It is enabled because a
# wrong bearing the operator can see beats no bearing at all -- but a lock
# acquired this way is a guess, and the logs mark it as one (acq_via).
CUELESS_INITIALISER = "rightmost_isolated"  # None = never acquire without a cue

CUE_STALE_S = 0.5        # no GCS_TARGET_BEARING for this long and the cue reads
                         # invalid, whatever the last one said. Matches the
                         # firmware's own LAT_TGT_TIMEOUT_S so both sides give up
                         # together. WITHOUT THIS the cue only goes invalid when a
                         # message ARRIVES saying so: if 42051 stops entirely the
                         # last bearing is held forever, and since the cue is the
                         # release authority, a frozen one drops a good lock as
                         # the target flies away from where the cue last was.

# --- initialisation/cue_nearest.py -----------------------------------------
# Acquire the detection nearest where the radar cue says the target is. The cue
# ACQUIRES and it VALIDATES; it never improves detection and is never fed back
# as a detection -- guidance already has it first-hand, at full rate, with no
# round trip, so echoing it would be a feedback loop (RPI_COMMS.md 2.1).
CUE_ACQUIRE_MAX_PX = 400.0   # a detection this far from the cue's pixel is not
                             # the cued target. Sized from the cue's own error
                             # budget, not the image: at focal 1516 this is
                             # 7.5 deg, covering a 0.5 s-old fix on a 30 m/s
                             # target at 1 km plus a couple of degrees of lens
                             # distortion, which is the largest uncorrected term
                             # in the whole bearing chain.
CUE_VALIDATE_WARN_DEG = 5.0  # warn when the tracked LOS and the cue disagree by
                             # more than this. NOT a veto: the cue is a 0.5 s-old
                             # radar fix reprojected through an uncalibrated
                             # principal point, so it is the less trustworthy of
                             # the two for fine angles. A persistent large
                             # residual means one of them is tracking the wrong
                             # object, which is what the comparison is for.

# --- initialisation/rightmost_isolated.py -----------------------------------
INIT_MIN_ISOLATION_PX = 100.0  # a candidate counts as isolated if its nearest
                               # neighbour among the frame's other detections is
                               # at least this far away.

# --- initialisation/manual_click.py -----------------------------------------
# The operator watches http://<pi>:8002/ (flight/preview.py) and clicks the
# target in the image. The click is a point, not a detection, so this still
# acquires the nearest real detector candidate to it -- same "acquire BETWEEN
# candidates the detector already found, never substitute for one" rule as
# cue_nearest, just sourced from a person instead of the radar.
MANUAL_CLICK_MAX_PX = 60.0     # a detection this far from the click is not what
                               # was pointed at. Tighter than CUE_ACQUIRE_MAX_PX:
                               # a click is a precise, un-aged pixel coordinate,
                               # not a 0.5 s-old reprojected radar fix, so there
                               # is no error budget to size this from -- it only
                               # needs to cover finger/mouse imprecision and the
                               # preview's own PREVIEW_SCALE rounding.
MANUAL_CLICK_TIMEOUT_S = 2.0   # how long a click stays armed waiting for a
                               # matching detection before it is dropped. Long
                               # enough to survive a frame or two of detector
                               # noise; short enough that an old click cannot
                               # cause a surprise lock long after the operator
                               # moved on to something else.

# ===========================================================================
# 5. TRACKER   (experiment/los_static_track.py: SmoothTracker)
# ===========================================================================
# Predicts the reference forward by attitude alone, then pulls it toward a
# detection inside a gate that widens with time and with angular rate.
TRACKER_BASE_GATE_PX = 50.0         # gate radius with a fresh fix, no rotation
TRACKER_GATE_PX_PER_FRAME = 10.0    # growth per frame since the last accepted fix
TRACKER_GATE_PX_PER_DEG_S = 1.5     # growth per deg/s of angular rate: both LOS
                                    # reprojection and detector confidence
                                    # degrade with rotation rate
TRACKER_MAX_GATE_PX = 350.0
TRACKER_OMEGA_REF_DEG_S = 100.0     # detector confidence is halved around this rate
TRACKER_CONF_REF = 50.0             # an SCR at or above this counts as fully
                                    # confident. This is what stops a
                                    # strong-but-farther detection automatically
                                    # beating a weak-but-closer one.
                                    # NOTE boxtophat_scr scores ~6% lower than
                                    # tophat_scr for the same target, so ~47
                                    # would keep the weighting identical if you
                                    # switch detectors and care about that.
TRACKER_ALPHA_MAX = 0.7             # hard cap on how far one frame may pull the
                                    # reference toward a measurement. This is
                                    # what makes a visible jump structurally
                                    # impossible rather than merely unlikely.
TRACKER_MISS_GAIN = 0.15            # extra pull allowed per consecutive miss
TRACKER_CROP_MARGIN_PX = 200.0      # added beyond the gate radius when sizing a
                                    # search crop, so the detector's own
                                    # morphological kernel does not react to the
                                    # crop's hard edge

# ===========================================================================
# 6. CAMERA MODEL AND MOUNT   (comms/bearing.py, comms/los.py)
# ===========================================================================
CAM_FOCAL_PX = 1516.0    # MEASURED field of view, not derived from lens specs:
                         # 45.8 deg across the 1280 px axis, 37.3 across 1024.
                         # Two earlier values were computed from a spec and both
                         # wrong in opposite directions -- 1108 under-reported
                         # bearings, 758 put a corner detection 18.8 deg out.
                         # Do NOT re-derive this from focal length and pixel
                         # pitch. Measure the field.
CAM_ROLL_DEG = 90.0      # image rotation about the boresight, +ve clockwise
                         # looking out along the view direction. MEASURED:
                         # predicted vs. phase-correlated observed image motion
                         # over 524 frame pairs gives corr +0.735 at +90 and
                         # -0.735 at -90 -- an exact negation, i.e. 180 deg about
                         # the boresight. Applied on THIS side, because the
                         # firmware models mount PITCH only.
CAM_PITCH_DEG = -25.0    # true mount pitch, +ve = boresight DOWN; the boresight
                         # is 25 deg up. HERE FOR REFERENCE AND CROSS-CHECKING
                         # ONLY -- it is applied by the Cube from LAT_CAM_PITCH
                         # and must never be pre-applied to anything on the wire,
                         # or it lands twice. comms/cube_link.py reads the live
                         # param so a disagreement is visible instead of assumed.

# Measured camera-to-body extrinsic rotation (R_bc): rotates a vector in the
# SENSOR frame (X=boresight, Y=image-right, Z=image-down) into BODY FRD
# (X=forward, Y=right, Z=down) -- the quaternion's own convention. It carries
# the roll (90) and the pitch (25 up) TOGETHER WITH the axis reorder from
# (right, down, forward) to (forward, right, down); it is NOT a pure roll
# composed with a pure pitch in image-axis order. Taken from a companion rig's
# extrinsics.json. comms/los.py asserts it is a proper rotation at startup,
# because a non-conformal mount matrix costs a silently skewed LOS, not an error.
MOUNT_R_BC = [
    [0.906307787037, -0.422618261741, 0.0],
    [0.0, 0.0, 1.0],
    [-0.422618261741, -0.906307787037, 0.0],
]

# ===========================================================================
# 7. TIMING: pairing a frame with an attitude
# ===========================================================================
CUBE_ATTITUDE_LOOKBACK_S = 0.1295
# How far BACK from a frame's stamp to look up attitude. Positive means into the
# past, the only direction that physically exists on a live feed.
#
# SUPERSEDED THE OLD TERM-BY-TERM DERIVATION (142.8 ms; see git history for
# that arithmetic) with a DIRECT MEASUREMENT, per row, at frame-complete:
# 146 ms at the top row, 113 ms at the bottom -- see CAM_ROW_LATENCY_TOP_S /
# _BOTTOM_S below. This constant is just their frame-centre average,
# (146 + 113) / 2 = 129.5 ms, kept as the single flat number for the two
# things that structurally need exactly one attitude sample per frame and
# have no particular row to prefer: SmoothTracker's own frame-to-frame
# prediction, and tools/live_track.py's default. Anywhere a specific
# detection's own row is known -- tools/flight_pipeline.py's final LOS send
# -- interpolate between the two row constants instead of using this flat one.
#
# WHICH STAMP THIS IS MEASURED FROM MATTERS MORE THAN THE ARITHMETIC.
# flight/camera.py stamps the instant cv2's read() RETURNS: frame complete, in
# userspace, after the whole frame -- every row -- has landed. That is what
# this constant and the two row constants below are all measured against.
#
# WAS 200 ms until 2026-09-10, a companion rig's empirical fit rather than
# derived. WAS 142.8 ms (115.0 ms world->first-byte + 35.4 ms bus transfer -
# 7.6 ms attitude-serialisation offset, all estimated, none measured per row)
# until superseded by the direct row measurement above on 2026-09-16.

CAM_ROW_LATENCY_TOP_S = 0.146     # MEASURED, not derived: how old the FIRST
                                  # row's content is, at the instant the whole
                                  # frame finishes landing in Pi memory (frame-
                                  # complete -- not when detect() gets to it).
CAM_ROW_LATENCY_BOTTOM_S = 0.113  # same, for the LAST row. Fresher, because it
                                  # was the last part of the frame written, so
                                  # less time has passed since ITS OWN capture
                                  # by the time the WHOLE frame is available.
                                  # Confirms, on this rig, the rolling-shutter
                                  # gradient README.md's "Open questions" could
                                  # only observe informally on a different one.
# Assumed LINEAR in row number between these two measured endpoints. Used in
# tools/flight_pipeline.py to date a SPECIFIC detection by its own row, on top
# of which the pipeline's own processing time (frame-complete -> about to
# send) is added -- that total is what the final LOS send and its logged
# capture timestamp use, in place of the flat CUBE_ATTITUDE_LOOKBACK_S above.

LOS_LATENCY_S = -0.000   # THE OFFLINE PATH ONLY: applied as `t - latency`
                         # against a recorded los-*.csv by
                         # experiment/los_static_track.py and the viewers.
                         # Being negative it looks 70 ms into the FUTURE of the
                         # frame stamp -- coherent there, where two separate
                         # clock series were tuned into agreement by eye, and
                         # meaningless live, where a future attitude does not
                         # exist yet and q_at() would clamp to "newest" every
                         # frame. Keeping it separate from the lookback above is
                         # what makes that failure impossible.
                         # NOTE recordings written by tools/flight_pipeline.py
                         # stamp the .rawrec and the CSV from ONE clock, so
                         # replaying those wants +0.1295 here; older recordings
                         # may not.
LOS_LATENCY_STEP_S = 0.005  # per key press while live-tuning with '[' / ']'
LOS_MIN_LATENCY_S = -0.5
LOS_MAX_LATENCY_S = 0.5

# ===========================================================================
# 8. THE CUBE LINK   (comms/)
# ===========================================================================
# Protocol contract: RPI_COMMS.md. Bring-up and wiring: deploy/COMMS.md.
CUBE_DEVICE = "/dev/ttyAMA0"   # RP1 uart0 on GPIO14/15 (header pins 8/10),
                               # enabled by dtparam=uart0=on. NOT /dev/serial0
                               # by name: on a Pi 5 that symlink points at the
                               # dedicated debug connector in some
                               # configurations. It happens to point at ttyAMA0
                               # on this box, which makes it a trap that works
                               # until the day it does not.
CUBE_BAUD = 57600              # MEASURED, and RPI_COMMS.md 1 agrees: the Cube's
                               # SERIAL2_BAUD is 57, not the 115 an older param
                               # file claims. A mismatch kills the link with no
                               # error at either end.
CUBE_SOURCE_SYSTEM = 254       # our sysid. Deliberately not 255, pymavlink's
                               # default: another MAVLink client on this host
                               # uses it, and two clients sharing a sysid makes
                               # a log unreadable.
CUBE_ATTITUDE_HZ = 30.0        # ATTITUDE_QUATERNION. 30 Hz x 44 B = 23% of the
                               # 5760 B/s link. It belongs to no stream group,
                               # so no Cube parameter can enable it -- it exists
                               # only if requested.
CUBE_CUE_HZ = 5.0              # GCS_TARGET_BEARING. The underlying radar cue
                               # does not update faster than this.
CUBE_RC_HZ = 5.0               # RC_CHANNELS, which drives the arming switches.
                               # 8x faster than RC_TIMEOUT_S, so a lost
                               # transmitter is noticed long before it matters,
                               # at 0.4% of the link.
CUBE_QUAT_KEEP = 400           # attitude ring length. At 30 Hz that is ~13 s of
                               # history, far more than any lookback reaches
                               # back through.
CUBE_HEARTBEAT_TIMEOUT_S = 10.0  # give up waiting for the first heartbeat
CUBE_ACK_TIMEOUT_S = 3.0         # COMMAND_ACK for a stream request. The ACK is
                                 # the ONLY feedback: a denial also emits a
                                 # STATUSTEXT, which is blocked on this private
                                 # channel, so nothing else tells you.
CUBE_PARAM_TIMEOUT_S = 3.0       # PARAM_VALUE for a parameter read
CUBE_RX_TIMEOUT_S = 1.0          # reader thread's recv_match timeout
CUBE_THREAD_JOIN_S = 2.0         # drain the reader before closing the port
CUBE_CAM_PITCH_PARAM = "LAT_CAM_PITCH"   # read, cross-checked, never applied here

# HOW LONG A COAST STILL COUNTS AS A DETECTION.
#
# The tracker reports "coasting" the moment a single frame puts nothing in its
# gate, and the uplink used to drop valid to 0 on that same frame. Measured on
# los-20260914-154321: 159 of 204 coast runs were 1-2 frames long -- a blink --
# yet each one told guidance "I have lost it", and a lock was held with no
# usable detection on 58% of frames.
#
# A coast is not nothing. The predicted LOS is a real detection propagated
# forward by measured attitude, so for a short interval it is the best fix
# available and better than declaring blindness. This is how long that stays
# true: valid holds through a coast for this long, measured from the last frame
# that actually fused a detection, then goes to 0.
#
# 0.5 s is deliberately the same number as the firmware's own LAT_DET_TIMEOUT_S
# and LAT_TGT_TIMEOUT_S deadmen, so the RPi gives up at the same moment the Cube
# would have given up on silence -- one timeout to reason about, not three.
#
# NOT A CUBE PARAMETER. It shares the LAT_ prefix with the firmware params above
# but lives entirely on this side of the wire; nothing reads or writes it on the
# Cube. Raising it trades a longer dead-reckoned fix for a longer window in which
# guidance steers on a target that may no longer be there.
LAT_DET_VALID_TIMEOUT = 0.5    # seconds of coasting still sent as valid=1

# ===========================================================================
# 9. THE RC SWITCHES   (flight/rc_arm.py)
# ===========================================================================
# Read straight off the RC_CHANNELS stream the autopilot already produces.
# Nothing has to be configured on the flight controller: no RCn_OPTION, no
# custom flight mode, no firmware change.
#
# TWO INDEPENDENT SWITCHES, and they must stay independent, so that all four
# combinations work: pipeline without recording, recording without pipeline,
# both, or neither. Recording must not depend on anything the tracker produces --
# footage taken with the algorithm off is how the algorithm gets improved.
ARM_RC_CHANNEL = 7          # ALGO switch: run detection, tracking and the uplink
ARM_RC_US = 1750.0          # ON above this pulse width
REC_RC_CHANNEL = 6          # RECORD switch: write a .rawrec episode
REC_RC_US = 1494.0          # ON above this; the midpoint of a 3-position switch
RC_INVERT_ARM = False       # set True if your switch's ON position reads ~982 us
RC_INVERT_REC = False       # rather than ~2006 us
RC_HYSTERESIS_US = 50.0     # Schmitt trigger: a switch must cross the threshold
                            # by this much to flip, not hover on it. Without it a
                            # noisy channel starts a new recording episode every
                            # frame, shredding a sortie into hundreds of files.
RC_TIMEOUT_S = 2.0          # no RC packet for this long counts as OFF, never as
                            # "hold the last value". Disarmed is the safe state,
                            # so anything unknown disarms: a seeker that keeps
                            # running because it stopped hearing the switch is
                            # the one failure this must not have.

# ===========================================================================
# 10. RECORDING   (flight/rawrec.py)
# ===========================================================================
# Raw is ~1966 MB/min at 25 fps -- 118 GB/hour -- so the disk, not the disk's
# speed, is the constraint.
RAW_RESERVE_MB = 2048       # a FLOOR of free space the recorder refuses to eat
                            # into, checked before opening an episode and again
                            # every RAW_DISK_CHECK_FRAMES while writing. Not an
                            # allocation: nothing is set aside. It converts "the
                            # disk filled and took the flight log with it" into
                            # "recording stopped, and said so, and the seeker
                            # kept flying". Counted in decimal MB, so 2048 is
                            # 2.05 GB.
RAW_RESERVE_MB_FALLBACK = 8192  # a bigger floor when writing to the SD card
                                # rather than the SSD, because that is the ROOT
                                # filesystem: running it dry does not merely lose
                                # the recording, it can take the system down.
RAW_QDEPTH_FRAMES = 8       # bounded queue between the camera thread and the
                            # writer. Full means the frame is DROPPED and
                            # counted: falling behind on disk must cost
                            # recording quality and nothing else -- never a
                            # dropped camera frame, never a stalled tracker.
                            # 8 frames is ~10.5 MB of buffer.
RAW_MAX_GB = 0.0            # hard cap per episode, 0 = none
RAW_DISK_CHECK_FRAMES = 200  # how often the writer re-checks free space. At
                             # ~79 MB/s it can overshoot the reserve by ~250 MB
                             # before noticing, so size the reserve with that
                             # slack in mind.
RAW_CLOSE_TIMEOUT_S = 10.0   # drain the writer queue on close

# ===========================================================================
# 11. THE FLIGHT PIPELINE   (tools/flight_pipeline.py)
# ===========================================================================
FLIGHT_STATUS_EVERY_S = 60.0   # seconds between [status] lines in the journal.
                               # The rate they report is measured over the LAST
                               # interval: a cumulative average divided by total
                               # runtime reads ~6x low after any spell of idling,
                               # which is exactly when it would mislead.
FLIGHT_CSV_FLUSH_FRAMES = 30   # ~1 s of frames. A power cut loses at most this
                               # many rows; the .rawrec beside it has no
                               # finalisation step and loses none.
FLIGHT_IDLE_SLEEP_S = 0.002    # main-loop sleep when no new frame has arrived

# ===========================================================================
# 12. VIEWERS AND TOOLS
# ===========================================================================
# --- shared UI: tools/rawrec_viewer.py, tools/web_viewer.py ----------------
VIEWER_INDEX_PROGRESS_INTERVAL_S = 1.0  # rawrec_viewer.build_index(): how often
                                        # to print a progress line while
                                        # scanning a file with no cached index
                                        # yet. A multi-GB capture over a slow
                                        # link (an external drive under load)
                                        # can take minutes; without this it is
                                        # indistinguishable from hung.
VIEWER_HUD_H = 30       # height in px of the info strip drawn above the frame
VIEWER_BOX_COLOR = (0, 200, 255)
VIEWER_LOS_TRACK_COLOR = (0, 255, 0)     # tracker fused a detection this frame
VIEWER_LOS_COAST_COLOR = (0, 165, 255)   # pure attitude dead-reckoning, no fix
VIEWER_SPEED_STEP = 1.5
VIEWER_MIN_SPEED = 1.0 / 16
VIEWER_MAX_SPEED = 16.0

# --- tools/web_viewer.py ---------------------------------------------------
WEB_HOST = "0.0.0.0"
WEB_PORT = 8000          # the SERVICE overrides this to 8001 via
                         # /etc/default/thermal-live, because another service on
                         # this host already binds 8000 and two things fighting
                         # over a port is a confusing failure in the field.
WEB_JPEG_QUALITY = 85

# --- tools/fb_viewer.py: the HDMI framebuffer preview ----------------------
FB_DEVICE = "/dev/fb0"
FB_MAX_FPS = 30.0        # the panel cannot show more than it refreshes and the
                         # blit is pure CPU

# --- releasing a bad lock, on the cue's evidence -----------------------------
# The tracker holds a reference until something tells it to let go. Nothing did,
# so a lock on the wrong object was permanent: it coasted on attitude for the
# rest of the flight, and because acquisition is cue-gated, a broken lock could
# never be replaced. These two make the RADAR CUE the release authority -- the
# one signal that is independent of the detector and the tracker both.
CUE_DROP_DEG = 10.0      # sustained separation between the tracked LOS and the
                         # cue beyond which the lock is presumed wrong.
                         # DELIBERATELY LARGER THAN THE ACQUISITION GATE:
                         # CUE_ACQUIRE_MAX_PX is 200 px = 7.5 deg, so a smaller
                         # value here would drop a lock the instant after
                         # acquiring it, and the pipeline would oscillate between
                         # acquire and drop forever.
CUE_DROP_FRAMES = 25     # consecutive disagreeing frames required before
                         # dropping. ~1 s at 25 fps. Persistence, not a single
                         # frame, because one bad cue or one bad blend must not
                         # cost a good track. NOTE the cue only updates at
                         # CUBE_CUE_HZ (5 Hz), so 25 frames is about 5
                         # INDEPENDENT cue samples, not 25.
                         #
                         # A DROP NEEDS POSITIVE EVIDENCE. The counter only
                         # advances while the cue is VALID and disagrees; an
                         # invalid cue resets nothing and drops nothing, because
                         # "no radar track" is not evidence that the tracker is
                         # wrong.

# --- flight/preview.py: the annotated live view -----------------------------
# A look at what the pipeline is actually seeing, served over HTTP so it works
# from any machine on the network with nothing installed.
PREVIEW_PORT = 8002        # not 8000 (taken on this host) and not
                           # WEB_PORT/8001 (tools/web_viewer.py), so the flight
                           # pipeline and a viewer can run side by side.
PREVIEW_MAX_FPS = 5.0      # ANNOTATION AND JPEG ENCODING HAPPEN ON THEIR OWN
                           # THREAD, but they still share four cores with a
                           # search pass that uses 32.6 ms of a 40 ms budget.
                           # 5 fps of preview is plenty to see what is going on
                           # and leaves the tracking loop alone. The main loop
                           # never waits for it: it hands over a reference and
                           # moves on, and the preview drops whatever it could
                           # not keep up with.
PREVIEW_SCALE = 0.5        # encode at half resolution. 640x512 is still every
                           # real sensor pixel -- the 1280x1024 stream is a 2x
                           # ISP upscale -- so this costs no information that
                           # was ever there, and quarters the encode.
PREVIEW_JPEG_QUALITY = 70
PREVIEW_CUE_COLOR = (255, 128, 0)      # where the radar says the target is
PREVIEW_DROP_COLOR = (0, 0, 255)       # a lock that was just given up

# --- tools/telemetry_viewer.py: replay of a flight's own logged los-*.csv --
# Draws what the deployed pipeline recorded per frame, alongside an
# independent LOS reprojection of our own, so the two can be compared.
TELEMETRY_COL_W = 235        # px per panel column. The panel columnises
                             # automatically: there are ~70 lines to show and
                             # they will not fit one column beside a scaled
                             # frame, and silently clipping the last groups
                             # (which include the comparison against our own
                             # reprojection) would defeat the point.
TELEMETRY_LINE_H = 17        # px, panel text line spacing
TELEMETRY_FONT_SCALE = 0.42
TELEMETRY_LOGGED_DROP_COLOR = (0, 0, 255)      # los_status == "dropped"
TELEMETRY_LOGGED_NONE_COLOR = (140, 140, 140)  # los_status == "none"
TELEMETRY_CUE_COLOR = (255, 0, 255)            # the Cube's cue, reprojected (cue_u/v)
TELEMETRY_OURS_COLOR = (255, 255, 0)           # our own independent LOS reprojection
TELEMETRY_ROI_COLOR = (120, 90, 0)             # the crop the pipeline searched

# --- sot/: single-object tracker, optional, selected on demand with 's' -----
# Independent of the detector/LOS pipeline entirely: drag a box around
# anything on any frame and this follows that patch of pixels forward by
# appearance alone. For a target the detector doesn't pick up, or as a
# second, independent check on where the LOS reprojection says it went.
SOT_ALGO = "opencv_csrt"    # chosen BY NAME from sot/ -- same convention as
                            # FLIGHT_INITIALISER / VIEWER_INITIALISER above.
TELEMETRY_SOT_MIN_BOX_PX = 4        # a drag smaller than this in either
                                    # dimension is a stray click, not a
                                    # selection -- ignored rather than handed
                                    # to the tracker as a near-zero-area box.
TELEMETRY_SOT_COLOR = (0, 255, 255)       # box while the tracker reports "found it"
TELEMETRY_SOT_LOST_COLOR = (0, 0, 255)    # box while it reports it lost the target
TELEMETRY_SOT_SELECT_COLOR = (255, 255, 255)  # the box being dragged out, live
