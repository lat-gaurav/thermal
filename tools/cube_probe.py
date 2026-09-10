#!/usr/bin/env python3
"""Walk RPI_COMMS.md section 8's bench checklist against the real Cube, in order.

    tools/cube_probe.py                    steps 1-7 (read-only, sends no detection)
    tools/cube_probe.py --send-detection    also step 8: put one test detection on the wire
    tools/cube_probe.py --seconds 20        watch attitude for longer

Each step is verifiable before the next, and each prints what it saw rather than
just pass/fail, because most of the failures here look identical from a distance:
a dead TX line, a missing request, a wrong baud, an unregenerated dialect and
firmware that does not implement a message all present as "nothing arrives".

WHY --send-detection IS OPT-IN. Everything else here only listens or asks for
telemetry. A DETECTION_TARGET_DATA is the one message that feeds guidance:
lat_intercept_note_detection() decodes it on any link in any flight mode, and in
MODE_INTERCEPT it steers. This tool refuses to send while the Cube reports ARMED
even with the flag, since a disarmed bench Cube is the only place a synthetic
bearing is unambiguously safe.

WHAT TO DO WITH STEP 8. The wire is only half the check: pull the Cube's LATD
dataflash message afterwards and confirm DAz/DEl equal what was sent and Det=1.
That is what proves the link, the message definition's CRC_EXTRA and the handler
all agree -- this side cannot see any of that, because nothing is acknowledged.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from comms.dialect import (mavutil, MAV, check as dialect_check,
                           MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                           MAVLINK_MSG_ID_GCS_TARGET_BEARING)
from comms.bearing import CamModel
from comms.cube_link import CubeLink
import config

PASS, FAIL, WARN = "  ok  ", " FAIL ", " warn "
_state = {"failed": 0}


def step(n, title):
    print("\n[%d] %s" % (n, title))


def report(kind, msg):
    if kind is FAIL:
        _state["failed"] += 1
    print("%s%s" % (kind, msg))


def ack_name(result):
    if result is None:
        return "no COMMAND_ACK came back"
    names = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
             3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
    return "%s (%d)" % (names.get(result, "?"), result)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default=config.CUBE_DEVICE)
    ap.add_argument("--baud", type=int, default=config.CUBE_BAUD)
    ap.add_argument("--seconds", type=float, default=6.0,
                    help="how long to measure the attitude stream (default 6)")
    ap.add_argument("--send-detection", action="store_true",
                    help="step 8: send ONE test DETECTION_TARGET_DATA (refused if armed)")
    ap.add_argument("--pixel", nargs=2, type=int, metavar=("U", "V"), default=(1000, 300),
                    help="pixel the step-8 test bearing is built from (default 1000 300)")
    args = ap.parse_args()

    print("cube_probe -- RPI_COMMS.md section 8 checklist")
    print("  target %s @ %d, our sysid %d" % (args.device, args.baud,
                                              config.CUBE_SOURCE_SYSTEM))

    # --- 2 (before 1, because every later failure looks the same if it is wrong) --
    step(2, "dialect and wire protocol")
    ok, detail = dialect_check()
    report(PASS if ok else FAIL, detail)
    if not ok:
        print("\nstopping: nothing below can be trusted until the dialect is right.")
        return 1

    # --- 1 ---------------------------------------------------------------------
    step(1, "serial port")
    if not os.path.exists(args.device):
        report(FAIL, "%s does not exist (dtparam=uart0=on in config.txt?)" % args.device)
        return 1
    report(PASS, "%s exists" % args.device)

    # --- 3 ---------------------------------------------------------------------
    step(3, "heartbeat")
    link = CubeLink(device=args.device, baud=args.baud, want_send=True)
    try:
        link.start()
    except (TimeoutError, RuntimeError) as e:
        report(FAIL, str(e))
        return 1
    report(PASS, "sysid %d component %d, ARMED=%s"
           % (link.sysid, link.compid, link.armed))
    report(PASS if link.can_send else FAIL,
           "detection_target_data_send is %savailable"
           % ("" if link.can_send else "NOT "))

    try:
        # --- 4 -----------------------------------------------------------------
        step(4, "LAT_CAM_PITCH")
        deadline = time.monotonic() + 3.0
        while link.cam_pitch_deg is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if link.cam_pitch_deg is None:
            report(WARN, "no PARAM_VALUE for LAT_CAM_PITCH in 3 s "
                         "(the bearing path does not need it; diagnostics do)")
        else:
            report(PASS, "LAT_CAM_PITCH = %+.2f deg (applied ON THE CUBE)"
                   % link.cam_pitch_deg)
            if abs(link.cam_pitch_deg - config.CAM_PITCH_DEG) > 0.51:
                report(WARN, "config.CAM_PITCH_DEG is %+.2f -- the two ends disagree "
                             "about this airframe's mount"
                       % config.CAM_PITCH_DEG)

        # --- 5 -----------------------------------------------------------------
        step(5, "ATTITUDE_QUATERNION at %.0f Hz" % link.attitude_hz)
        report(PASS if link.ack_results.get("ATTITUDE_QUATERNION") == 0 else WARN,
               "SET_MESSAGE_INTERVAL ack: %s"
               % ack_name(link.ack_results.get("ATTITUDE_QUATERNION")))
        t0 = time.monotonic()
        n0 = link.n_attitude
        time.sleep(args.seconds)
        got = link.n_attitude - n0
        hz = got / (time.monotonic() - t0)
        if got == 0:
            report(FAIL, "no ATTITUDE_QUATERNION in %.0f s" % args.seconds)
        else:
            report(PASS if hz > link.attitude_hz * 0.7 else WARN,
                   "%d messages in %.1f s = %.1f Hz (asked %.0f)"
                   % (got, args.seconds, hz, link.attitude_hz))
            q = link.q_at(time.monotonic() - 0.2)
            oldest, newest, n = link.attitude_span()
            report(PASS, "ring %d samples spanning %.1f s; q_at(now-200ms) = "
                         "(%+.4f, %+.4f, %+.4f, %+.4f)"
                   % (n, newest - oldest, q[0], q[1], q[2], q[3]))
            nrm = math.sqrt(sum(c * c for c in q))
            report(PASS if abs(nrm - 1.0) < 1e-6 else FAIL,
                   "SLERPed quaternion norm = %.9f" % nrm)

        # --- 6 -----------------------------------------------------------------
        step(6, "GCS_TARGET_BEARING one-shot (id 42051)")
        before = (link.cue.n_valid, link.cue.n_invalid)
        res = link.request_once(MAVLINK_MSG_ID_GCS_TARGET_BEARING)
        report(PASS if res == 0 else WARN, "REQUEST_MESSAGE ack: %s" % ack_name(res))
        deadline = time.monotonic() + 2.0
        while (link.cue.n_valid, link.cue.n_invalid) == before and time.monotonic() < deadline:
            time.sleep(0.02)
        got_msg = (link.cue.n_valid, link.cue.n_invalid) != before
        if got_msg:
            if link.cue.valid:
                report(PASS, "cue VALID: az %+.2f el %+.2f deg, range %.1f m, age %d ms"
                       % (math.degrees(link.cue.az), math.degrees(link.cue.el),
                          link.cue.rng, link.cue.age_ms))
            else:
                report(PASS, "cue arrived with gcs_target_valid=0 -- the correct 'no cue' "
                             "answer on a bench with no radar and no GPS. The request/"
                             "decode path works.")
        elif res == 0:
            report(FAIL, "ACCEPTED ack but no 42051 arrived. The ack means the firmware "
                         "knows the message, so suspect the CRC_EXTRA in "
                         "comms/mavlink/gcs_target_bearing.msg.xml, which is a "
                         "reconstruction -- see its header comment.")
        else:
            report(WARN, "no 42051 and the ack was %s -- this firmware build most "
                         "likely does not implement it" % ack_name(res))

        # --- 7 -----------------------------------------------------------------
        step(7, "GCS_TARGET_BEARING as a stream, then stopped")
        res = link.request_interval(MAVLINK_MSG_ID_GCS_TARGET_BEARING, args.seconds and 5.0)
        report(PASS if res == 0 else WARN, "SET_MESSAGE_INTERVAL(5 Hz) ack: %s" % ack_name(res))
        before = (link.cue.n_valid, link.cue.n_invalid)
        time.sleep(2.0)
        n = sum((link.cue.n_valid, link.cue.n_invalid)) - sum(before)
        report(PASS if n else WARN, "%d messages in 2.0 s (%.1f Hz)" % (n, n / 2.0))
        res = link.request_interval(MAVLINK_MSG_ID_GCS_TARGET_BEARING, 0)  # -1 internally
        report(PASS if res == 0 else WARN, "stop (-1) ack: %s" % ack_name(res))

        # --- 8 -----------------------------------------------------------------
        step(8, "DETECTION_TARGET_DATA uplink")
        if not args.send_detection:
            report(WARN, "skipped -- pass --send-detection to actually transmit one")
        elif link.armed:
            report(FAIL, "REFUSED: the Cube reports ARMED. This message feeds guidance; "
                         "send it only to a disarmed bench Cube.")
        else:
            cam = CamModel()
            u, v = args.pixel
            az, el = cam.az_el(u, v)
            sent = link.send_detection(az, el, valid=True, confidence=0.9,
                                       capture_usec=int(time.time() * 1e6))
            report(PASS if sent else FAIL,
                   "sent pixel (%d,%d) -> az %+.4f deg, el %+.4f deg"
                   % (u, v, math.degrees(az), math.degrees(el)))
            report(WARN, "NOT confirmed from this side -- nothing acknowledges it. Check "
                         "the Cube's LATD log: DAz=%.6f DEl=%.6f Det=1"
                   % (az, el))

        print("\nstatus: %s" % link.status())
    finally:
        link.close()

    print("\n%s" % ("all steps passed" if not _state["failed"]
                    else "%d step(s) FAILED" % _state["failed"]))
    return 1 if _state["failed"] else 0


if __name__ == "__main__":
    sys.exit(main())
