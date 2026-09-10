#!/usr/bin/env python3
"""The whole pipeline on the live camera, with live attitude from the Cube.

    tools/live_track.py                       track and uplink, status to stdout
    tools/live_track.py --no-uplink           track only, send nothing to the Cube
    tools/live_track.py --seconds 30          run for 30 s then stop
    tools/live_track.py --csv out.csv         log every frame

WHAT THIS ADDS OVER tools/web_viewer.py --live. The web viewer runs the detector
on a live feed but cannot run the LOS half of the pipeline: SmoothTracker and
filters/los_proximity need a per-frame camera attitude, which until now existed
only as a recorded los-*.csv beside a .rawrec, so a live session reported
has_los=false and built no tracker. comms/cube_link.py supplies that attitude
live at 30 Hz on the same CLOCK_MONOTONIC the frames are stamped with, so the
full chain runs here:

    grab -> attitude lookup -> LOS predict -> ROI crop -> detect -> filter
         -> tracker blend -> bearing -> DETECTION_TARGET_DATA

THE ROI CROP IS THE POINT, not an optimisation. tophat_scr costs ~0.70 s on a
full 1280x1024 frame on this Pi -- 17x the 40 ms budget at 25 fps. The tracker's
own gate is what makes a crop possible, and the crop is what makes the rate
usable, so LOS and affordability arrive together or not at all. Before the
tracker has a reference the search is necessarily full-frame and slow; that cost
is paid once, at acquisition.

INDEX-ALIGNED HISTORY. SmoothTracker indexes quats[] and frame_times[] by frame
number, so this keeps two lists that grow by one entry per PROCESSED frame and
hands the same list objects to the tracker. They are never trimmed: an index
into them has to stay valid for the life of the run, and a trim would silently
re-point every stored reference. ~100 B per frame, so an hour at 4 fps is
about 1.4 MB.

WHAT IT SENDS. One DETECTION_TARGET_DATA per processed frame, camera-frame az/el
with mount pitch NOT applied (the Cube applies LAT_CAM_PITCH), including
valid=0 frames -- silence for 0.5 s is a guidance timeout, valid=0 is
information. See RPI_COMMS.md section 3.
"""
import argparse
import csv
import math
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

import rawrec_viewer as rv          # detector/filter/initialiser loaders, LOS helpers
import web_viewer as wv             # LiveSource: the camera grab thread
import config
from comms.bearing import CamModel
from comms.cube_link import CubeLink


class StampedSource(wv.LiveSource):
    """LiveSource that records WHEN each frame was dequeued.

    The parent serves "whatever is latest" with no timestamp, which is right for
    a viewer and wrong here: an unstamped frame could be anything up to a frame
    period old by the time the main loop looks at it, and 40 ms of unknown age
    is the same order as the 200 ms camera latency this pipeline is trying to
    compensate for. Stamping in the grab thread, immediately after read()
    returns, removes that unknown.
    """

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                t = time.monotonic()
                with self.lock:
                    self.latest = (t, frame)
            else:
                time.sleep(0.01)

    def get_stamped(self):
        with self.lock:
            return self.latest


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/thermal0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=1024)
    ap.add_argument("--detector", default=None,
                    help="module name from detector/ (default: the first one found)")
    ap.add_argument("--no-uplink", action="store_true",
                    help="run the pipeline but send nothing to the Cube")
    ap.add_argument("--cube", default=config.CUBE_DEVICE)
    ap.add_argument("--baud", type=int, default=config.CUBE_BAUD)
    ap.add_argument("--lookback", type=float, default=config.CUBE_ATTITUDE_LOOKBACK_S,
                    help="seconds to look BACK for the attitude of a frame "
                         "(default %(default)s; see config.py for why this is not "
                         "LOS_LATENCY_S)")
    ap.add_argument("--focal", type=float, default=config.CAM_FOCAL_PX)
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after N seconds")
    ap.add_argument("--csv", help="append one row per processed frame here")
    ap.add_argument("--quiet", action="store_true", help="no per-frame status line")
    args = ap.parse_args()

    # ---- the pipeline's own modules, loaded the same way the viewers load them ----
    detectors = dict(rv.load_detectors())
    if not detectors:
        sys.exit("no detector found in detector/")
    det_name = args.detector or sorted(detectors)[0]
    if det_name not in detectors:
        sys.exit("unknown detector %r; available: %s" % (det_name, ", ".join(sorted(detectors))))
    detect = detectors[det_name]
    filters = rv.load_filters()
    initialisers = rv.load_initialisers()
    los = rv._load_los_track()

    w, h = args.width, args.height
    cx, cy = w / 2.0, h / 2.0
    cam = CamModel(focal_px=args.focal, cx=cx, cy=cy, width=w, height=h)

    # ---- the Cube link ----------------------------------------------------------
    print("[live_track] connecting to the Cube on %s @ %d ..." % (args.cube, args.baud))
    link = CubeLink(device=args.cube, baud=args.baud, want_send=not args.no_uplink)
    link.start()
    # LAT_CAM_PITCH is requested during start() but the reply takes a moment; wait
    # for it before printing, because "None" on this line reads as "the param does
    # not exist" when it only means "not yet".
    _deadline = time.monotonic() + 2.0
    while link.cam_pitch_deg is None and time.monotonic() < _deadline:
        time.sleep(0.05)
    print("[live_track] Cube sysid %d, ARMED=%s, LAT_CAM_PITCH=%s, uplink=%s"
          % (link.sysid, link.armed,
             "not reported" if link.cam_pitch_deg is None
             else "%+.2f deg (applied on the CUBE)" % link.cam_pitch_deg,
             "off (--no-uplink)" if args.no_uplink else
             ("on" if link.can_send else "UNAVAILABLE (dialect)")))

    # Wait for the attitude ring to cover the lookback, or the first frames get a
    # clamped attitude and the tracker anchors against a stale one.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        oldest, newest, n = link.attitude_span()
        if n >= 2 and (newest - oldest) >= args.lookback:
            break
        time.sleep(0.05)
    oldest, newest, n = link.attitude_span()
    if n < 2:
        link.close()
        sys.exit("no attitude from the Cube -- run tools/cube_probe.py")
    print("[live_track] attitude ring: %d samples, %.1f s span, %.1f Hz"
          % (n, newest - oldest, link.attitude_hz_measured()))

    # ---- the camera -------------------------------------------------------------
    print("[live_track] opening %s ..." % args.device)
    src = StampedSource(args.device, w, h)

    # ---- index-aligned history, shared BY REFERENCE with the tracker ------------
    frame_times, quats = [], []
    tracker = los.SmoothTracker(quats, frame_times, args.focal, cx, cy, los.project)

    writer = fh = None
    if args.csv:
        fh = open(args.csv, "a", newline="")
        writer = csv.writer(fh)
        if fh.tell() == 0:
            writer.writerow(["idx", "t_mono", "qw", "qx", "qy", "qz", "status",
                             "roi", "n_boxes", "los_x", "los_y", "az_deg", "el_deg",
                             "valid", "det_ms", "loop_ms"])

    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    idx = -1
    last_obj = None
    t_run = time.monotonic()
    n_det = n_valid = 0
    print("[live_track] detector=%s filters=%s init=%s  lookback=%.3fs"
          % (det_name, ",".join(n for n, _ in filters) or "-",
             ",".join(n for n, _ in initialisers) or "-", args.lookback))
    print("[live_track] running; Ctrl-C to stop")

    try:
        while not stop:
            if args.seconds and time.monotonic() - t_run >= args.seconds:
                break
            stamped = src.get_stamped()
            if stamped is None or stamped is last_obj:
                time.sleep(0.002)
                continue
            last_obj = stamped
            t_frame, raw = stamped
            t_loop = time.monotonic()

            q = link.q_at(t_frame - args.lookback)
            if q is None:
                continue

            # Append BEFORE using idx: the tracker indexes these by frame number.
            idx += 1
            frame_times.append(t_frame)
            quats.append(q)

            # ---- ROI crop from the tracker's own gate, or full frame ----
            roi = "FULL"
            offset = (0, 0)
            view = raw
            if tracker.ref is not None:
                predicted = tracker.point(idx)
                gate = tracker.gate_px(idx)
                bounds = (los.crop_bounds(predicted[0], predicted[1],
                                          gate + los.CROP_MARGIN_PX, w, h)
                          if predicted is not None and gate is not None else None)
                if bounds is not None:
                    x0, y0, x1, y1 = bounds
                    view = raw[y0:y1, x0:x1]
                    offset = (x0, y0)
                    roi = "%dx%d" % (x1 - x0, y1 - y0)

            t0 = time.monotonic()
            boxes = detect(view)
            det_ms = 1000.0 * (time.monotonic() - t0)
            if offset != (0, 0):
                boxes = [(x + offset[0], y + offset[1], bw, bh, s)
                         for (x, y, bw, bh, s) in boxes]
            n_det += 1

            # ---- acquire, or advance ----
            if tracker.ref is None:
                kept = boxes
                for _, fn in filters:
                    kept = fn(kept, {"los_point": None, "frame_w": w, "frame_h": h})
                uv = (rv.pick_initialiser(config.VIEWER_INITIALISER, initialisers)[1](
                          kept, {"frame_w": w, "frame_h": h}) if initialisers else None)
                if uv is not None:
                    tracker.set_click(idx, uv)
            else:
                tracker.update(idx, boxes)

            los_point = tracker.point(idx)
            status = tracker.status if tracker.ref is not None else "none"

            # ---- uplink ----
            valid = status == "tracking" and los_point is not None
            az_deg = el_deg = ""
            if los_point is not None:
                az, el = cam.az_el(los_point[0], los_point[1])
                az_deg, el_deg = math.degrees(az), math.degrees(el)
                link.send_detection(az, el, valid=valid, confidence=1.0 if valid else 0.0,
                                    capture_usec=int(t_frame * 1e6))
            else:
                link.send_detection(0.0, 0.0, valid=False, confidence=0.0,
                                    capture_usec=int(t_frame * 1e6))
            if valid:
                n_valid += 1

            loop_ms = 1000.0 * (time.monotonic() - t_loop)
            if writer:
                writer.writerow([idx, "%.6f" % t_frame, "%.6f" % q[0], "%.6f" % q[1],
                                 "%.6f" % q[2], "%.6f" % q[3], status, roi, len(boxes),
                                 "" if los_point is None else "%.1f" % los_point[0],
                                 "" if los_point is None else "%.1f" % los_point[1],
                                 "" if az_deg == "" else "%.4f" % az_deg,
                                 "" if el_deg == "" else "%.4f" % el_deg,
                                 int(valid), "%.1f" % det_ms, "%.1f" % loop_ms])
            if not args.quiet:
                el = time.monotonic() - t_run
                sys.stdout.write(
                    "\r[%6.1fs] f%-5d %-8s roi=%-9s box=%-2d det=%6.1fms "
                    "loop=%6.1fms %5.2f fps  up=%d/%d  att=%.1fHz  "
                    % (el, idx, status, roi, len(boxes), det_ms, loop_ms,
                       n_det / el if el > 0 else 0, n_valid, link.n_sent,
                       link.attitude_hz_measured()))
                sys.stdout.flush()
    finally:
        # Drain the grab thread before teardown: it is parked in a blocking
        # cap.read(), and letting the interpreter tear down around it aborts the
        # process with "FATAL: exception not rethrown".
        src.running = False
        src.thread.join(timeout=2.0)
        src.cap.release()
        if fh is not None:
            fh.close()
        el = max(time.monotonic() - t_run, 1e-9)
        print("\n[live_track] %d frames in %.1f s = %.2f fps; %d tracking, "
              "%d messages sent (%.1f Hz)"
              % (n_det, el, n_det / el, n_valid, link.n_sent, link.n_sent / el))
        if link.n_sent and link.n_sent / el < 2.0:
            print("[live_track] WARNING: under 2 Hz on the wire. LAT_DET_TIMEOUT_S is "
                  "0.5 s, so guidance will treat detection as stale between frames.")
        print("[live_track] link: %s" % link.status())
        link.close()


if __name__ == "__main__":
    main()
