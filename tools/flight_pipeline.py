#!/usr/bin/env python3
"""The flight pipeline: this repo's detection and tracking, in the operational
envelope this rig already flew under.

    flight_pipeline.py --out-csv logs/los-S.csv --raw-video logs/flight-S.rawrec
    flight_pipeline.py --no-uplink            run everything, transmit nothing
    flight_pipeline.py --always-on            ignore the RC switches, run immediately

WHAT IS THE SAME as the service this replaces, deliberately, so a sortie behaves
the way the crew already expects:

  TWO INDEPENDENT RC SWITCHES.  ch7 runs the algorithm, ch6 records. All four
  combinations work, because recording must not depend on anything the tracker
  produces -- footage taken with the algorithm off is how the algorithm gets
  improved. Both fail safe: no RC for 2 s is OFF, never "hold the last value".

  EPISODES.  Every switch-on of ch6 opens a new `-NN.rawrec`, every switch-off
  closes it. The CSV and the raw files of one flight share a timestamp, supplied
  by the caller, so the pair is unambiguous without guessing.

  A CLEAN TRACKER ON EVERY RE-ARM.  A tracker resumed after idling would still
  hold the previous episode's reference and gate. It is rebuilt from scratch.

  RECORDING IS FED FROM THE CAMERA THREAD, so a raw capture is complete even
  while detection is too slow to look at every frame.

  IT KEEPS FLYING THROUGH A FAILED RECORDING.  A full card, or a breached
  reserve, logs loudly and leaves the tracker running. Losing the recording is
  bad; losing the seeker is worse.

WHAT IS DIFFERENT, which is the whole point:

  DETECTION is this repo's -- detector/ (boxtophat_scr by default), gated by
  filters/, initialised by initialisation/.
  TRACKING is experiment/los_static_track.py's SmoothTracker: an attitude-only
  LOS prediction, corrected by detections inside a rate-widened gate, blended
  with a hard cap on how far one frame may pull it.
  The ROI CROP comes from that tracker's own gate, which is what makes
  full-rate detection affordable at all.

WHAT IS NOT HERE, and you should know before flying it:

  No stabilisation, no ego-motion compensation, no horizon crop, and no
  centroid/peak mode latch. The old pipeline had all four. This tracker
  compensates for camera rotation by reprojecting its reference through the
  attitude, which is a different approach to the same problem, not a port of
  that one.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import threading
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)
sys.path.insert(0, _HERE)

import rawrec_viewer as rv          # detector/filter/initialiser loaders + LOS helpers
import config
from comms.bearing import CamModel
from comms.cube_link import CubeLink
from comms.los import LosSolver
from flight.camera import FlightCamera
from flight.rawrec import RawRecWriter
from flight.preview import Preview
from flight.rc_arm import RcArm

CSV_HEADER = [
    # los-*.csv compatibility: t_mono + qw..qz are what experiment/los_static_track.py
    # and tools/rawrec_viewer.py look up to replay a recording with the LOS overlay.
    # Keep these four names and this clock, or a recorded flight cannot be reviewed.
    "frame", "t_mono", "t_wall", "qw", "qx", "qy", "qz", "att_age_ms",
    # operational state
    "algo", "arm_reason", "rc_arm_us", "recording", "rec_reason", "rc_rec_us",
    "episode", "rec_frames",
    # detection and tracking
    "roi", "n_boxes", "n_kept", "los_x", "los_y", "los_status", "gate_px",
    # WHY the tracker did what it did. It computes all of this to make its
    # decision; without it a log that says "coasting" cannot tell you whether
    # nothing was in the gate or nothing was good enough.
    "n_in_gate", "det_score", "match_dist_px", "alpha", "omega_deg_s", "misses",
    # the release path: how long the cue has been disagreeing, and how many
    # locks have been given up so far
    "cue_bad_frames", "drops",
    # what went on the wire
    "az_deg", "el_deg", "det_valid", "seq_sent",
    # the final LOS: camera mounting applied in full, then the vehicle attitude.
    # body_az/el are the same direction measured from the airframe's nose, which
    # is what a human reading this log wants; los_n/e/d are the world-frame unit
    # vector, and are what went out in the message's los_n/e/d fields.
    "body_az_deg", "body_el_deg", "los_n", "los_e", "los_d",
    # the cue, and the running comparison against it. cue_resid_deg is the
    # angular separation between the tracked LOS and the cue -- logged, never
    # acted on.
    "cue_valid", "cue_az_deg", "cue_el_deg", "cue_range_m", "cue_age_ms",
    "cue_u", "cue_v", "cue_resid_deg",
    # link and hardware health, per frame, so a degradation can be located in
    # time rather than inferred afterwards
    "att_hz", "hb_age_ms", "cube_armed", "grabbed", "raw_dropped",
    # rec_idx maps THIS row onto the recorded video. Row order alone does not:
    # the processing path skips frames the recorder still captured, and the
    # episode column keeps its value after recording stops.
    "rec_idx",
    # cost
    "det_ms", "loop_ms", "skipped",
]


def git_revision(repo):
    """(sha, dirty) of the working tree that is about to fly, or (None, None).

    A sortie's log is only interpretable against the code that produced it, and
    "the working tree on the Pi that afternoon" is not a version. `dirty` matters
    as much as the sha: a clean sha can be checked out again, a dirty one cannot,
    and knowing which you are looking at is the difference between reproducing a
    flight and guessing at it.
    """
    try:
        sha = subprocess.run(["git", "-C", repo, "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        st = subprocess.run(["git", "-C", repo, "status", "--porcelain"],
                            capture_output=True, text=True, timeout=5)
        if sha.returncode != 0:
            return None, None
        return sha.stdout.strip(), bool(st.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        return None, None


def write_session_meta(path, args, det_name, init_name, link, extra=None):
    """Dump every parameter this sortie actually ran with, beside the CSV.

    WHY A SIDECAR AND NOT JUST THE JOURNAL. A flight log is read months later,
    usually by someone asking "why did it do that", and the answer is often a
    threshold that has since been changed. The CSV records what happened; this
    records what the pipeline believed at the time -- every config value, every
    resolved argument, the detector and initialiser that actually loaded, the
    firmware's own LAT_CAM_PITCH, and the software versions. Without it, a log
    is only interpretable against a working tree that no longer exists.

    Best-effort: a sortie must never fail to start because a metadata file
    could not be written.
    """
    try:
        import cv2 as _cv2
        cfg = {k: getattr(config, k) for k in sorted(dir(config)) if k.isupper()}
        sha, dirty = git_revision(_ROOT)
        meta = {
            "written_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "t_mono_at_write": time.monotonic(),
            "argv": sys.argv,
            "args": {k: v for k, v in sorted(vars(args).items())},
            "loaded": {"detector": det_name, "initialiser": init_name},
            "code": {"git_sha": sha, "dirty": dirty, "repo": _ROOT},
            "cube": {"sysid": link.sysid, "armed": link.armed,
                     "lat_cam_pitch_deg": link.cam_pitch_deg,
                     "stream_acks": getattr(link, "ack_results", {}),
                     "device": link.device, "baud": link.baud},
            "host": {"python": sys.version.split()[0], "opencv": _cv2.__version__,
                     "uname": " ".join(os.uname()), "cpu_temp_c": cpu_temp_c(),
                     "throttled_at_start": throttled_word()},
            "config": cfg,
        }
        if extra:
            meta.update(extra)
        with open(path, "w") as fh:
            json.dump(meta, fh, indent=2, sort_keys=True, default=str)
        return path
    except Exception as e:                       # never block a sortie on this
        print("[meta] could not write %s: %s" % (path, e), flush=True)
        return None


def now_el(t0):
    return time.monotonic() - t0


def cpu_temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as fh:
            return int(fh.read()) / 1000.0
    except OSError:
        return float("nan")


def throttled_word():
    """The firmware's throttle word. 0x0 means nothing has been limited.

    Sampled into the status line rather than per frame: this Pi soft-throttles
    near 80 C, and a sortie that throttled halfway through has two different
    performance regimes in one log. Without this you cannot tell which.
    """
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=3).stdout.strip()
        return out.split("=")[-1] if "=" in out else "?"
    except (OSError, subprocess.SubprocessError):
        return "?"


def stamp_paths(args):
    """Resolve the episode-0 names. The caller supplies the shared stamp."""
    return args.out_csv, args.raw_video


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default=config.CAMERA_DEVICE)
    ap.add_argument("--width", type=int, default=config.CAMERA_WIDTH)
    ap.add_argument("--height", type=int, default=config.CAMERA_HEIGHT)
    ap.add_argument("--detector", default=None,
                    help="module from detector/ (default: first found)")
    ap.add_argument("--initialiser", default=config.FLIGHT_INITIALISER,
                    help="module from initialisation/ that acquires the target "
                         "(default %(default)s)")
    ap.add_argument("--cue-fallback", action="store_true",
                    help="FAILURE CASE ONLY, off by default: when the tracker has no "
                         "lock at all, send the cue's own bearing instead of nothing. "
                         "RPI_COMMS.md 2.1 warns against this -- guidance already has "
                         "the cue first-hand, so echoing it is a feedback loop. Here "
                         "for when the detector is the thing that has failed.")
    ap.add_argument("--cube", default=config.CUBE_DEVICE)
    ap.add_argument("--baud", type=int, default=config.CUBE_BAUD)
    ap.add_argument("--focal", type=float, default=config.CAM_FOCAL_PX)
    ap.add_argument("--lookback", type=float, default=config.CUBE_ATTITUDE_LOOKBACK_S)
    ap.add_argument("--out-csv", help="per-frame log (one row per processed frame)")
    ap.add_argument("--raw-video", metavar="PATH",
                    help="base name for .rawrec episodes; -NN is appended per episode")
    ap.add_argument("--raw-reserve-mb", type=int, default=config.RAW_RESERVE_MB,
                    help="stop recording before the card has less than this free")
    ap.add_argument("--raw-max-gb", type=float, default=config.RAW_MAX_GB,
                    help="0 = no cap")
    ap.add_argument("--no-uplink", action="store_true",
                    help="run everything, send nothing to the Cube")
    ap.add_argument("--always-on", action="store_true",
                    help="ignore the RC switches: algo on, and record if --raw-video")
    ap.add_argument("--arm-chan", type=int, default=config.ARM_RC_CHANNEL)
    ap.add_argument("--arm-us", type=float, default=config.ARM_RC_US)
    ap.add_argument("--rec-chan", type=int, default=config.REC_RC_CHANNEL)
    ap.add_argument("--rec-us", type=float, default=config.REC_RC_US)
    ap.add_argument("--preview-port", type=int, default=config.PREVIEW_PORT,
                    help="annotated live view over HTTP; 0 disables it")
    ap.add_argument("--status-every", type=float, default=config.FLIGHT_STATUS_EVERY_S,
                    help="seconds between [status] lines (default 60)")
    args = ap.parse_args()

    # ---- the pipeline's own modules, loaded the way every other tool loads them ----
    detectors = dict(rv.load_detectors())
    if not detectors:
        sys.exit("no detector found in detector/")
    det_name = args.detector or sorted(detectors)[0]
    if det_name not in detectors:
        sys.exit("unknown detector %r; have: %s" % (det_name, ", ".join(sorted(detectors))))
    detect = detectors[det_name]
    filters = rv.load_filters()
    initialisers = rv.load_initialisers()
    init_name, init_fn = rv.pick_initialiser(args.initialiser, initialisers)
    if init_fn is None:
        sys.exit("no initialiser found in initialisation/")
    los = rv._load_los_track()

    W, H = args.width, args.height
    cx, cy = W / 2.0, H / 2.0
    cam = CamModel(focal_px=args.focal, cx=cx, cy=cy, width=W, height=H)
    # The full-mount LOS solver. Separate from CamModel on purpose: CamModel
    # applies the mount ROLL only, because the Cube applies the pitch itself;
    # this applies the mount in FULL, because a world-frame LOS has no second
    # party to finish the job.
    losv = LosSolver(focal_px=args.focal, cx=cx, cy=cy, width=W, height=H)
    if not losv.is_conformal():
        sys.exit("config.MOUNT_R_BC is not a proper rotation -- every LOS derived "
                 "from it would be silently skewed. Fix the extrinsics.")

    print("=" * 72)
    print("flight pipeline")
    print("  camera      %s  %dx%d" % (args.camera, W, H))
    print("  detector    %s   filters %s"
          % (det_name, ",".join(n for n, _ in filters) or "-"))
    print("  acquire on  %s   cue fallback %s"
          % (init_name, "ON (failure case)" if args.cue_fallback else "off"))
    print("  focal       %.0f px   attitude lookback %.0f ms"
          % (args.focal, 1000 * args.lookback))

    # ---- the Cube: attitude down, RC down, detections up, one connection ----
    link = CubeLink(device=args.cube, baud=args.baud, want_send=not args.no_uplink)
    link.start()
    deadline = time.monotonic() + 2.0
    while link.cam_pitch_deg is None and time.monotonic() < deadline:
        time.sleep(0.05)
    print("  cube        %s @ %d  sysid %d  ARMED=%s  LAT_CAM_PITCH=%s"
          % (args.cube, args.baud, link.sysid, link.armed,
             "not reported" if link.cam_pitch_deg is None
             else "%+.2f deg (applied ON THE CUBE)" % link.cam_pitch_deg))
    print("  uplink      %s" % ("OFF (--no-uplink): nothing is sent" if args.no_uplink
                                else ("ON" if link.can_send else "UNAVAILABLE (dialect)")))
    for k, v in sorted(getattr(link, "ack_results", {}).items()):
        print("              %-22s stream ack %s" % (k, "ACCEPTED" if v == 0 else v))

    # ---- the two switches ----
    if args.always_on:
        algo_arm = rec_arm = None
        print("  switches    --always-on: RC ignored, algo runs immediately")
    else:
        algo_arm = RcArm(args.arm_chan, args.arm_us, invert=config.RC_INVERT_ARM, name="algo")
        rec_arm = RcArm(args.rec_chan, args.rec_us, invert=config.RC_INVERT_REC, name="record")
        print("  switches    algo %s   record %s   (fail safe: no RC for %.0fs = OFF)"
              % (algo_arm.describe(), rec_arm.describe(), config.RC_TIMEOUT_S))

    # ---- attitude history has to cover the lookback before the first frame ----
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        o, n_, c = link.attitude_span()
        if c >= 2 and (n_ - o) >= args.lookback:
            break
        time.sleep(0.05)
    o, n_, c = link.attitude_span()
    if c < 2:
        link.close()
        sys.exit("no attitude from the Cube -- run tools/cube_probe.py")
    print("  attitude    %d samples, %.1f s span, %.1f Hz" % (c, n_ - o,
                                                              link.attitude_hz_measured()))

    # ---- camera ----
    src = FlightCamera(args.camera, W, H).start()
    preview = None
    if args.preview_port:
        preview = Preview(port=args.preview_port).start()
        print("  preview     http://0.0.0.0:%d/   (%.0f fps, %.0f%% scale, own thread)"
              % (args.preview_port, preview.max_fps, 100 * preview.scale))
    print("  recording   %s" % (args.raw_video or "OFF (no --raw-video)"))
    print("  csv         %s" % (args.out_csv or "OFF (no --out-csv)"))
    print("=" * 72, flush=True)

    csv_f = csv_w = None
    if args.out_csv:
        csv_f = open(args.out_csv, "w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(CSV_HEADER)
        # The configuration this sortie ran with, beside the data it produced.
        mp = write_session_meta(os.path.splitext(args.out_csv)[0] + ".meta.json",
                                args, det_name, init_name, link)
        if mp:
            print("  meta        %s" % mp, flush=True)
    _sha, _dirty = git_revision(_ROOT)
    print("  code        %s%s" % (_sha[:12] if _sha else "not a git repo",
                                  "  *** UNCOMMITTED CHANGES ***" if _dirty else " (clean)"),
          flush=True)

    # ---- index-aligned attitude history, shared BY REFERENCE with the tracker ----
    frame_times, quats = [], []
    tracker = los.SmoothTracker(quats, frame_times, args.focal, cx, cy, los.project)

    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def open_episode(ep):
        """Open one .rawrec episode and hand it to the camera thread."""
        if not args.raw_video:
            return None
        root, ext = os.path.splitext(args.raw_video)
        path = args.raw_video if ep <= 0 else "%s-%02d%s" % (root, ep, ext or ".rawrec")
        try:
            r = RawRecWriter(path, W, H, bpp=1, pixfmt="GREY",
                             reserve_mb=args.raw_reserve_mb, max_gb=args.raw_max_gb,
                             meta={"source": args.camera, "episode": ep,
                                   "focal": args.focal, "detector": det_name,
                                   "out_csv": args.out_csv or "",
                                   "advertised_fps": float(config.CAMERA_FPS)}).open()
        except OSError as e:
            # Out of space, or the reserve would be breached. Loudly, and keep flying:
            # a missing recording must never take the seeker down with it.
            print("\n[raw] CANNOT RECORD: %s" % e, flush=True)
            return None
        src.sink = r
        print("\n[raw] RECORDING  %s  (~%.0f MB/s at %d fps)"
              % (path, W * H * config.CAMERA_FPS / 1e6, config.CAMERA_FPS), flush=True)
        return r

    def close_episode(r):
        """Detach the recorder and finish it on a BACKGROUND thread.

        close() drains the write queue and fsync()s the file. MEASURED: doing
        that on the main loop stalled detection for 9.6 SECONDS when a 0.92 GB
        episode closed on the SD card, skipping 239 frames -- i.e. flipping the
        record switch off froze the seeker for ten seconds, mid-sortie. The
        fsync is not optional (it is what guarantees the tail of the file
        reached the card) but it has no business happening between two frames.

        Detaching the sink is the only part that must be synchronous: once
        src.sink is None the camera thread stops offering frames, so the
        background close sees a queue that is no longer growing.
        """
        if r is None:
            return None
        src.sink = None
        def _finish():
            st = r.close()
            print("\n[raw] SAVED  %s  %d frames, %.2f GB, %d dropped%s"
                  % (st["path"], st["frames"], st["gb"], st["dropped"],
                     "  [" + st["stop_reason"] + "]" if st["stop_reason"] else ""),
                  flush=True)
        t = threading.Thread(target=_finish, name="rawrec-close", daemon=True)
        t.start()
        closing.append(t)
        return None

    closing = []          # background episode-close threads, joined at shutdown
    armed = algo_arm is None
    arm_reason = "always_on" if algo_arm is None else "startup"
    recording = False
    rec_reason = "startup"
    rec = None
    episode = 0
    if algo_arm is None and args.raw_video:
        episode = 1
        rec = open_episode(episode)
        recording = rec is not None
        rec_reason = "always_on"

    idx = -1
    n_proc = n_valid = n_idle = 0
    t_run = time.monotonic()
    t_status = t_run
    # Counted at the last status line, so the rate reported is the rate over the
    # LAST interval. A cumulative average divided by total runtime reads ~6x low
    # after any spell of idling, which is exactly when it would mislead.
    n_proc_at_status = n_valid_at_status = 0
    # THE RATE IS MEASURED FROM CONSECUTIVE PROCESSED FRAMES, not from a frame
    # count over wall time. Dividing by elapsed time counts every second the
    # algorithm was disarmed as a second it was running slowly: with 221 s idle
    # and 55 s armed, a true 25 fps was reported as 5.1. An EWMA over the actual
    # inter-frame dt cannot be fooled that way, and it also reacts to a real
    # slowdown within a second instead of averaging it away.
    fps_ewma = None
    t_prev_frame = None
    total_skipped = 0
    n_cue_disagree = n_cue_sent = 0
    cue_bad_frames = 0
    # vcgencmd forks a process, so sample it on the status cadence and cache it
    # for the page rather than running it once per frame.
    throttle_cache = [throttled_word()]
    git_sha, git_dirty = git_revision(_ROOT)
    det_ms = loop_ms = 0.0

    try:
        while not stop:
            t_frame, frame, skipped = src.read()
            if frame is None:
                time.sleep(config.FLIGHT_IDLE_SLEEP_S)
                continue
            t_loop = time.monotonic()

            # ---- RC: two independent switches -------------------------------
            rc_arm_us = rc_rec_us = ""
            if rec_arm is not None and args.raw_video:
                want_rec, rec_reason = rec_arm.update(link)
                rc_rec_us = rec_arm.last_us if rec_arm.last_us is not None else ""
                if want_rec != recording:
                    # `recording` tracks WHAT THE SWITCH SAYS, not whether a file
                    # opened. If open_episode() fails -- full card, breached reserve
                    # -- it has already said so loudly, and the switch state must
                    # still latch here or the transition re-fires every frame,
                    # burning a new episode number each time and spamming the log.
                    recording = want_rec
                    if recording:
                        episode += 1
                        rec = open_episode(episode)
                    else:
                        rec = close_episode(rec)
            elif rec_arm is not None:
                # No --raw-video: the record switch is inert. Still read it, so the
                # CSV records what the crew's switch was doing.
                _w, rec_reason = rec_arm.update(link)
                rc_rec_us = rec_arm.last_us if rec_arm.last_us is not None else ""
            if algo_arm is not None:
                want_algo, arm_reason = algo_arm.update(link)
                rc_arm_us = algo_arm.last_us if algo_arm.last_us is not None else ""
                if want_algo != armed:
                    armed = want_algo
                    if armed:
                        # A tracker resumed after idling would still hold the last
                        # episode's reference and gate. Start clean.
                        frame_times.clear()
                        quats.clear()
                        tracker = los.SmoothTracker(quats, frame_times, args.focal,
                                                    cx, cy, los.project)
                        idx = -1
                        print("\n[algo] RUNNING  ch%d=%sus" % (algo_arm.chan,
                                                               algo_arm.last_us), flush=True)
                    else:
                        print("\n[algo] stopped (%s) after %d frames"
                              % (arm_reason, n_proc), flush=True)

            # ---- algo off: the recorder is already fed by the camera thread ----
            if not armed:
                n_idle += 1
                if csv_w:
                    row = [""] * len(CSV_HEADER)
                    row[0:3] = ["", "%.6f" % t_frame, "%.6f" % time.time()]
                    row[8:16] = [0, arm_reason, rc_arm_us, int(recording), rec_reason,
                                 rc_rec_us, episode, rec.frames if rec else ""]
                    csv_w.writerow(row)
                if preview is not None:
                    # The panel must be populated while IDLE too -- this is the
                    # state the crew watches BEFORE arming, and a page that only
                    # fills in once the algorithm runs is blank exactly when it
                    # is being used to decide whether to arm. Everything not
                    # produced by the detection path reads None rather than
                    # being absent, so the layout does not jump when it arms.
                    preview.offer(frame, {"hud": [
                        "ALGO OFF (%s)  rec %s ep%d" % (
                            arm_reason, "on" if rec is not None else "off", episode),
                        "raw feed; detection and tracking are not running"]},
                        state={
                            "frame": None, "fps": None, "det_ms": None,
                            "loop_ms": None, "roi": "-", "skipped": total_skipped,
                            "grabbed": src.grabbed,
                            "uptime_s": round(now_el(t_run), 1),
                            "status": "algo off", "los_x": None, "los_y": None,
                            "gate_px": None, "n_in_gate": None, "det_score": None,
                            "match_dist_px": None, "alpha": None,
                            "omega_deg_s": None, "misses": None,
                            "drops": getattr(tracker, "drops", 0),
                            "last_release": getattr(tracker, "last_release", None),
                            "detector": det_name, "initialiser": init_name,
                            "n_boxes": None, "n_kept": None,
                            "az_deg": None, "el_deg": None, "det_valid": False,
                            "seq_sent": link.n_sent, "body_az_deg": None,
                            "body_el_deg": None, "los_n": None, "los_e": None,
                            "los_d": None,
                            "cue_valid": bool(link.cue.valid),
                            "cue_az_deg": (math.degrees(link.cue.az)
                                           if link.cue.valid else None),
                            "cue_el_deg": (math.degrees(link.cue.el)
                                           if link.cue.valid else None),
                            "cue_range_m": link.cue.rng if link.cue.valid else None,
                            "cue_age_ms": link.cue.age_ms if link.cue.valid else None,
                            "cue_u": None, "cue_v": None, "cue_resid_deg": None,
                            "cue_bad_frames": cue_bad_frames,
                            "cue_seen": "%d valid / %d invalid" % (
                                link.cue.n_valid, link.cue.n_invalid),
                            "sysid": link.sysid, "cube_armed": link.armed,
                            "att_hz": round(link.attitude_hz_measured(), 2),
                            "att_age_ms": None,
                            "hb_age_ms": None if link.t_heartbeat is None else
                            round(1000.0 * (time.monotonic() - link.t_heartbeat)),
                            "lat_cam_pitch_deg": link.cam_pitch_deg,
                            "uplink": "on" if link.can_send else "off",
                            "n_sent": link.n_sent,
                            "algo_armed": False, "arm_reason": arm_reason,
                            "rc_arm_us": rc_arm_us if rc_arm_us != "" else None,
                            "rec_armed": bool(recording), "rec_reason": rec_reason,
                            "rc_rec_us": rc_rec_us if rc_rec_us != "" else None,
                            "rec_on": rec is not None, "episode": episode,
                            "rec_frames": rec.frames if rec else None,
                            "raw_dropped": rec.dropped if rec else None,
                            "rec_path": os.path.basename(rec.path) if rec else None,
                            "rec_stop_reason": rec.stop_reason if rec else None,
                            "cpu_temp_c": round(cpu_temp_c(), 1),
                            "throttled": throttle_cache[0],
                            "git_sha": (git_sha or "")[:12], "dirty": git_dirty,
                            "lookback_s": args.lookback, "focal_px": args.focal,
                            "cue_acquire_px": config.CUE_ACQUIRE_MAX_PX,
                            "cue_drop_deg": config.CUE_DROP_DEG,
                            "cue_drop_frames": config.CUE_DROP_FRAMES,
                            "min_scr": config.DETECTOR_MIN_SCR,
                        })
                now = time.monotonic()
                if now - t_status >= args.status_every:
                    print("\n[status] %s  %df idle  rec=%s ep%d %s  att %.1fHz"
                          % ("REC-ONLY" if rec is not None else "IDLE", n_idle,
                             ("on" if rec is not None else
                              ("SWITCH-ON/NO-FILE" if recording else "off")), episode,
                             "%df" % rec.frames if rec else "", link.attitude_hz_measured()),
                          flush=True)
                    t_status = now
                continue

            # ---- attitude for this frame, from the ring ----
            q = link.q_at(t_frame - args.lookback)
            if q is None:
                continue
            # How stale the newest attitude sample is at this frame. Not the same as
            # the lookback: this is link health, that is camera latency.
            _o, newest, _c = link.attitude_span()
            att_age_ms = 1000.0 * (t_frame - newest) if newest else 0.0

            if t_prev_frame is not None:
                dt = t_frame - t_prev_frame
                if 0.0 < dt < 2.0:        # ignore an arm transition or a stall
                    inst = 1.0 / dt
                    fps_ewma = inst if fps_ewma is None else 0.9 * fps_ewma + 0.1 * inst
            t_prev_frame = t_frame
            total_skipped += skipped

            idx += 1
            frame_times.append(t_frame)
            quats.append(q)

            # ---- ROI crop from the tracker's own gate ----
            roi, offset, view, gate = "FULL", (0, 0), frame, ""
            if tracker.ref is not None:
                pred = tracker.point(idx)
                gate = tracker.gate_px(idx)
                b = (los.crop_bounds(pred[0], pred[1], gate + los.CROP_MARGIN_PX, W, H)
                     if pred is not None and gate is not None else None)
                if b is not None:
                    x0, y0, x1, y1 = b
                    view, offset = frame[y0:y1, x0:x1], (x0, y0)
                    roi = "%dx%d" % (x1 - x0, y1 - y0)

            t0 = time.monotonic()
            boxes = detect(view)
            det_ms = 1000.0 * (time.monotonic() - t0)
            if offset != (0, 0):
                boxes = [(x + offset[0], y + offset[1], bw, bh, s)
                         for (x, y, bw, bh, s) in boxes]

            # ---- the radar cue, projected into this frame ----
            # An INVALID cue must arrive downstream as None, never as its zeroed
            # az/el: zero is a legal bearing (dead ahead), so passing it through
            # would acquire whatever happens to sit near the boresight.
            cue_point = None
            cue_az = cue_el = cue_rng = cue_age = ""
            if link.cue.valid:
                cue_point = cam.pixel_of(link.cue.az, link.cue.el)
                cue_az, cue_el = math.degrees(link.cue.az), math.degrees(link.cue.el)
                cue_rng, cue_age = link.cue.rng, link.cue.age_ms

            ctx = {"los_point": None, "frame_w": W, "frame_h": H,
                   "cue_point": cue_point}

            # ---- acquire, or advance ----
            los_point = None
            if tracker.ref is None:
                kept = boxes
                for _, fn in filters:
                    kept = fn(kept, ctx)
                uv = init_fn(kept, ctx)
                if uv is not None:
                    tracker.set_click(idx, uv)
                    print("\n[acq] locked on a detection %s"
                          % ("%.0f px from the cue" % ((uv[0] - cue_point[0]) ** 2
                                                       + (uv[1] - cue_point[1]) ** 2) ** 0.5
                             if cue_point else "(no cue)"), flush=True)
                n_kept = len(kept)
            else:
                # The filters run here too. Applying them only at acquisition let
                # the tracker fuse raw detector output -- streaks and frame-border
                # artifacts included -- for the whole rest of the lock.
                kept = boxes
                for _, fn in filters:
                    kept = fn(kept, ctx)
                tracker.update(idx, kept)
                n_kept = len(kept)
            los_point = tracker.point(idx)
            status = tracker.status if tracker.ref is not None else "none"
            tl = tracker.last

            # ---- uplink ----
            valid = status == "tracking" and los_point is not None
            az_deg = el_deg = ""
            cue_resid_deg = ""
            baz_deg = bel_deg = ""
            los_n = los_e = los_d = ""
            if los_point is not None:
                # what the Cube CONSUMES: camera frame, mount pitch left to it
                az, el = cam.az_el(los_point[0], los_point[1])
                az_deg, el_deg = math.degrees(az), math.degrees(el)
                # the FINAL LOS: mount applied in full, then this frame's attitude
                e_ned = losv.ned(los_point[0], los_point[1], q)
                los_n, los_e, los_d = (float(e_ned[0]), float(e_ned[1]), float(e_ned[2]))
                baz, bel = losv.body_az_el(los_point[0], los_point[1])
                baz_deg, bel_deg = math.degrees(baz), math.degrees(bel)
                # ---- CONSTANT VALIDATION against the cue --------------------
                # The cue and the tracker are two independent answers to "where
                # is the target". Their angular separation is logged every frame
                # and NOTHING is corrected by it: the cue is a 0.5 s-old radar
                # fix reprojected through an uncalibrated principal point, so it
                # is the less trustworthy of the two for fine angles. A
                # persistent large residual means one of them is tracking the
                # wrong object, which is the failure this is here to expose.
                if link.cue.valid:
                    dot = (math.cos(el) * math.cos(az) * math.cos(link.cue.el) * math.cos(link.cue.az)
                           + math.cos(el) * math.sin(az) * math.cos(link.cue.el) * math.sin(link.cue.az)
                           + math.sin(el) * math.sin(link.cue.el))
                    cue_resid_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
                    if cue_resid_deg > config.CUE_VALIDATE_WARN_DEG:
                        n_cue_disagree += 1

                    # ---- THE CUE IS THE RELEASE AUTHORITY ----------------
                    # Close enough: nothing to do, and the counter resets, so
                    # disagreement has to be SUSTAINED rather than merely
                    # sampled. Far apart for CUE_DROP_FRAMES: the lock is
                    # presumed wrong and given up, which lets the next frame
                    # acquire afresh against the cue. This is the only thing in
                    # the pipeline that can say the tracker is wrong, because it
                    # is the only signal independent of both the detector and
                    # the tracker.
                    if cue_resid_deg > config.CUE_DROP_DEG:
                        cue_bad_frames += 1
                    else:
                        cue_bad_frames = 0
                    if cue_bad_frames >= config.CUE_DROP_FRAMES:
                        print("\n[drop] released the lock: %.1f deg from the cue for "
                              "%d frames (limit %.1f deg / %d frames). Re-acquiring."
                              % (cue_resid_deg, cue_bad_frames,
                                 config.CUE_DROP_DEG, config.CUE_DROP_FRAMES), flush=True)
                        tracker.release("cue disagreement %.1f deg" % cue_resid_deg)
                        cue_bad_frames = 0
                        # Nothing is tracked any more, so this frame must not
                        # claim otherwise: the bearing still goes out (it is the
                        # last thing we believed) but valid=0 tells guidance not
                        # to steer on it.
                        valid = False
                        status = "dropped"
                else:
                    # An invalid cue is not evidence that the tracker is wrong.
                    # Do not advance the counter and do not reset it either --
                    # a cue that flickers in and out must not launder a
                    # disagreement away.
                    pass
                link.send_detection(az, el, valid=valid, confidence=1.0 if valid else 0.0,
                                    capture_usec=int(t_frame * 1e6),
                                    los_ned=(los_n, los_e, los_d))
            elif args.cue_fallback and link.cue.valid:
                # FAILURE CASE ONLY. The detector has produced nothing to track,
                # so the cue's own bearing goes out instead. valid stays 0: this
                # is not a detection and must never be counted as one.
                link.send_detection(link.cue.az, link.cue.el, valid=False,
                                    confidence=0.0, capture_usec=int(t_frame * 1e6))
                n_cue_sent += 1
            else:
                link.send_detection(0.0, 0.0, valid=False, confidence=0.0,
                                    capture_usec=int(t_frame * 1e6))
            n_proc += 1
            if valid:
                n_valid += 1
            loop_ms = 1000.0 * (time.monotonic() - t_loop)

            # ---- the annotated view. offer() stores a reference and returns;
            # all the drawing and encoding happens on the preview's own thread,
            # because a full-frame search leaves only ~7 ms of the 40 ms budget
            # and a JPEG does not fit in it.
            if preview is not None:
                el_s = now_el(t_run)
                preview.offer(frame, {
                    "boxes": kept,
                    "los_point": los_point,
                    "status": status,
                    "gate_px": gate if gate != "" else None,
                    "cue_point": cue_point,
                    # One line burnt into the image, so a saved /frame.jpg
                    # identifies itself. Everything else goes to the page's
                    # panel, where it can be read as numbers instead of
                    # squinted at over the picture.
                    "hud": ["f%d  %s  %s" % (idx, status, roi)],
                }, state={
                    # --- pipeline
                    "frame": idx, "fps": None if fps_ewma is None else round(fps_ewma, 2),
                    "det_ms": round(det_ms, 1), "loop_ms": round(loop_ms, 1),
                    "roi": roi, "skipped": total_skipped, "grabbed": src.grabbed,
                    "uptime_s": round(el_s, 1),
                    # --- tracker, including WHY it decided what it did
                    "status": status,
                    "los_x": None if los_point is None else round(los_point[0], 1),
                    "los_y": None if los_point is None else round(los_point[1], 1),
                    "gate_px": None if gate == "" else round(gate, 1),
                    "n_in_gate": tl.get("n_in_gate"),
                    "det_score": tl.get("score"), "match_dist_px": tl.get("dist_px"),
                    "alpha": tl.get("alpha"), "omega_deg_s": tl.get("omega_deg_s"),
                    "misses": tracker.misses, "drops": tracker.drops,
                    "last_release": getattr(tracker, "last_release", None),
                    # --- detector
                    "detector": det_name, "initialiser": init_name,
                    "n_boxes": len(boxes), "n_kept": n_kept,
                    # --- what went on the wire
                    "az_deg": az_deg if az_deg != "" else None,
                    "el_deg": el_deg if el_deg != "" else None,
                    "det_valid": bool(valid), "seq_sent": link.n_sent,
                    "body_az_deg": baz_deg if baz_deg != "" else None,
                    "body_el_deg": bel_deg if bel_deg != "" else None,
                    "los_n": los_n if los_n != "" else None,
                    "los_e": los_e if los_e != "" else None,
                    "los_d": los_d if los_d != "" else None,
                    # --- the radar cue, and the running comparison against it
                    "cue_valid": bool(link.cue.valid),
                    "cue_az_deg": cue_az if cue_az != "" else None,
                    "cue_el_deg": cue_el if cue_el != "" else None,
                    "cue_range_m": cue_rng if cue_rng != "" else None,
                    "cue_age_ms": cue_age if cue_age != "" else None,
                    "cue_u": None if cue_point is None else round(cue_point[0], 1),
                    "cue_v": None if cue_point is None else round(cue_point[1], 1),
                    "cue_resid_deg": cue_resid_deg if cue_resid_deg != "" else None,
                    "cue_bad_frames": cue_bad_frames,
                    "cue_seen": "%d valid / %d invalid" % (link.cue.n_valid,
                                                            link.cue.n_invalid),
                    # --- link health
                    "sysid": link.sysid, "cube_armed": link.armed,
                    "att_hz": round(link.attitude_hz_measured(), 2),
                    "att_age_ms": round(att_age_ms, 1),
                    "hb_age_ms": None if link.t_heartbeat is None else
                    round(1000.0 * (time.monotonic() - link.t_heartbeat)),
                    "lat_cam_pitch_deg": link.cam_pitch_deg,
                    "uplink": "on" if link.can_send else "off",
                    "n_sent": link.n_sent,
                    # --- the switches
                    "algo_armed": bool(armed), "arm_reason": arm_reason,
                    "rc_arm_us": rc_arm_us if rc_arm_us != "" else None,
                    "rec_armed": bool(recording), "rec_reason": rec_reason,
                    "rc_rec_us": rc_rec_us if rc_rec_us != "" else None,
                    # --- recording
                    "rec_on": rec is not None, "episode": episode,
                    "rec_frames": rec.frames if rec else None,
                    "raw_dropped": rec.dropped if rec else None,
                    "rec_path": os.path.basename(rec.path) if rec else None,
                    "rec_stop_reason": rec.stop_reason if rec else None,
                    # --- host
                    "cpu_temp_c": round(cpu_temp_c(), 1),
                    "throttled": throttle_cache[0],
                    "git_sha": (git_sha or "")[:12], "dirty": git_dirty,
                    # --- the settings in force, so the page is self-describing
                    "lookback_s": args.lookback, "focal_px": args.focal,
                    "cue_acquire_px": config.CUE_ACQUIRE_MAX_PX,
                    "cue_drop_deg": config.CUE_DROP_DEG,
                    "cue_drop_frames": config.CUE_DROP_FRAMES,
                    "min_scr": config.DETECTOR_MIN_SCR,
                })

            if csv_w:
                csv_w.writerow([
                    idx, "%.6f" % t_frame, "%.6f" % time.time(),
                    "%.6f" % q[0], "%.6f" % q[1], "%.6f" % q[2], "%.6f" % q[3],
                    "%.1f" % att_age_ms,
                    1, arm_reason, rc_arm_us, int(recording), rec_reason, rc_rec_us,
                    episode, rec.frames if rec else "",
                    roi, len(boxes), n_kept,
                    "" if los_point is None else "%.1f" % los_point[0],
                    "" if los_point is None else "%.1f" % los_point[1],
                    status, "" if gate == "" else "%.0f" % gate,
                    tl.get("n_in_gate", ""),
                    "" if tl.get("score") is None else "%.1f" % tl["score"],
                    "" if tl.get("dist_px") is None else "%.1f" % tl["dist_px"],
                    "" if tl.get("alpha") is None else "%.3f" % tl["alpha"],
                    "" if tl.get("omega_deg_s") is None else "%.2f" % tl["omega_deg_s"],
                    tracker.misses, cue_bad_frames, tracker.drops,
                    "" if az_deg == "" else "%.4f" % az_deg,
                    "" if el_deg == "" else "%.4f" % el_deg,
                    int(valid), link.n_sent,
                    "" if baz_deg == "" else "%.4f" % baz_deg,
                    "" if bel_deg == "" else "%.4f" % bel_deg,
                    "" if los_n == "" else "%.6f" % los_n,
                    "" if los_e == "" else "%.6f" % los_e,
                    "" if los_d == "" else "%.6f" % los_d,
                    int(bool(link.cue.valid)),
                    "" if cue_az == "" else "%.4f" % cue_az,
                    "" if cue_el == "" else "%.4f" % cue_el,
                    "" if cue_rng == "" else "%.1f" % cue_rng,
                    "" if cue_age == "" else "%d" % cue_age,
                    "" if cue_point is None else "%.1f" % cue_point[0],
                    "" if cue_point is None else "%.1f" % cue_point[1],
                    "" if cue_resid_deg == "" else "%.3f" % cue_resid_deg,
                    "%.2f" % link.attitude_hz_measured(),
                    "" if link.t_heartbeat is None
                    else "%.0f" % (1000.0 * (time.monotonic() - link.t_heartbeat)),
                    "" if link.armed is None else int(link.armed),
                    src.grabbed, rec.dropped if rec else "",
                    rec.frames - 1 if rec else "",
                    "%.1f" % det_ms, "%.1f" % loop_ms, skipped,
                ])
                if n_proc % config.FLIGHT_CSV_FLUSH_FRAMES == 0:
                    csv_f.flush()

            now = time.monotonic()
            if now - t_status >= args.status_every:
                d_v = n_valid - n_valid_at_status
                d_n = n_proc - n_proc_at_status
                print("\n[status] RUNNING %df total  %.1ffps  proc %.1fms  track %d/%d  "
                      "roi=%s  rec=%s ep%d %s  up %d  att %.1fHz  cue=%s  "
                      "skipped %d  drops %d  %.1fC thr=%s"
                      % (n_proc, fps_ewma or 0.0, loop_ms, d_v, d_n,
                         roi, ("on" if rec is not None else
                               ("SWITCH-ON/NO-FILE" if recording else "off")), episode,
                         "%df" % rec.frames if rec else "", link.n_sent,
                         link.attitude_hz_measured(),
                         ("%.1fdeg" % cue_resid_deg if cue_resid_deg != ""
                          else ("valid,no-track" if link.cue.valid else "none")),
                         total_skipped, tracker.drops, cpu_temp_c(),
                         throttle_cache[0]), flush=True)
                t_status = now
                n_proc_at_status, n_valid_at_status = n_proc, n_valid
                throttle_cache[0] = throttled_word()
    finally:
        rec = close_episode(rec)
        for t in closing:
            t.join(timeout=config.RAW_CLOSE_TIMEOUT_S + 2.0)
        src.stop()
        if preview is not None:
            preview.stop()
        if csv_f is not None:
            csv_f.close()
        el_s = max(time.monotonic() - t_run, 1e-9)
        print("\n[done] %d processed (%.2f fps), %d tracking, %d idle, "
              "%d grabbed, %d sent, %d lock(s) dropped"
              % (n_proc, n_proc / el_s, n_valid, n_idle, src.grabbed, link.n_sent,
                 getattr(tracker, "drops", 0)))
        print("[done] link: %s" % link.status())
        link.close()


if __name__ == "__main__":
    main()
