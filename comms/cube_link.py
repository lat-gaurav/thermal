"""The Cube link: ATTITUDE_QUATERNION down, DETECTION_TARGET_DATA up.

    from comms.cube_link import CubeLink
    link = CubeLink()
    link.start()                       # blocks until the first heartbeat
    q = link.q_at(time.monotonic() - 0.2)   # attitude 200 ms ago, SLERPed
    link.send_detection(az, el, valid=True, confidence=0.9)

NOTHING ARRIVES UNTIL YOU ASK. TELEM2 streams nothing but a 1 Hz heartbeat
(MAV3_POSITION and MAV3_EXTRA1 are both 0) and ATTITUDE_QUATERNION belongs to no
stream group at all, so no Cube parameter can ever turn it on -- only
SET_MESSAGE_INTERVAL. A connection that shows only heartbeats is correct
behaviour, not a fault (RPI_COMMS.md section 4).

INTERVALS ARE PER-CHANNEL AND IN RAM ONLY, so every Cube reboot drops them and
start() re-requests unconditionally. Re-requesting one that is already running
is harmless, which matters here: another MAVLink client on this host may have
asked for the same stream already, and this link must neither depend on that nor
be confused by it.

WHY THE ATTITUDE RING EXISTS. A frame's timestamp is when its bytes reached the
host, roughly 92-95 ms after the photons (see README.md). The attitude that
matters is the attitude at exposure, so every lookup reaches BACKWARDS by the
camera latency -- which means keeping history and interpolating, not reading
"the latest quaternion". q_at() SLERPs between the two bracketing samples;
nearest-neighbour was tried on the recorded path and went stale on ~30% of
frames.

CLOCKS. Samples are stamped with time.monotonic() ON RECEIPT, which is the same
CLOCK_MONOTONIC the frame timestamps use, so the two are directly comparable.
The stamp is late by the message's own transit: 44 B at 57600 baud is 7.6 ms of
serialisation plus scheduling. That is a systematic lag inside the ring, and it
is absorbed by the latency term the caller subtracts (config.LOS_LATENCY_S,
tuned against observed motion) rather than corrected here, because a guess at
the split between the two would not be measurable separately.
"""
import importlib.util
import math
import pathlib
import sys
import threading
import time

# MUST precede any pymavlink import; see comms/dialect.py.
from comms.dialect import (mavutil, MAV, check as dialect_check,
                           MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                           MAVLINK_MSG_ID_GCS_TARGET_BEARING)

MAVLINK_MSG_ID_RC_CHANNELS = 65

import numpy as np

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import config

CAM_PITCH_PARAM = config.CUBE_CAM_PITCH_PARAM


def _load_slerp():
    """Reuse experiment/los_static_track.py's slerp rather than keeping a second copy.

    config.py's own docstring is about exactly this failure: the same idea
    existing as two independently maintained copies. That module is loaded by
    file path because experiment/ is not a package, the same way
    tools/rawrec_viewer.py already loads it.
    """
    spec = importlib.util.spec_from_file_location(
        "los_static_track", _REPO_ROOT / "experiment" / "los_static_track.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.slerp


class TargetCue:
    """The latest usable GCS_TARGET_BEARING: the radar cue, in camera frame.

    A SEARCH PRIOR, NOT A MEASUREMENT. Never send it back as a detection --
    guidance already has this cue first-hand, at full rate, with no round trip
    (RPI_COMMS.md section 2.1).
    """

    def __init__(self):
        self.az = self.el = self.rng = None
        self.age_ms = None
        self.t_usec = None
        self.t_recv = None
        self.n_valid = 0
        self.n_invalid = 0

    @property
    def valid(self):
        return self.az is not None

    def update(self, msg):
        self.t_recv = time.monotonic()
        if msg.gcs_target_valid != 1:
            # az/el/range are then all exactly zero. Zero is a legal bearing --
            # dead ahead -- so it must never be mistaken for one.
            self.az = self.el = self.rng = None
            self.age_ms = None
            self.n_invalid += 1
            return False
        self.az, self.el = float(msg.gcs_target_az), float(msg.gcs_target_el)
        self.rng = float(msg.gcs_target_range)
        self.age_ms = int(msg.gcs_target_age_ms)
        self.t_usec = int(msg.time_usec)
        self.n_valid += 1
        return True

    def search_window_rad(self, tgt_speed_mps=30.0, base_rad=math.radians(3.0)):
        """Half-width to search around (az, el), widened for cue age and closeness.

        age_ms is bounded to 0..499 whenever valid, because a cue older than
        LAT_TGT_TIMEOUT_S = 0.5 s reads invalid instead. So this widens the
        window WITHIN that half second; it never has to cover a minutes-old cue.
        base_rad is your own radar/mount uncertainty and is not something the
        Cube can tell you.
        """
        if not self.valid:
            return None
        drift_m = tgt_speed_mps * (self.age_ms / 1000.0)
        return base_rad + (drift_m / max(self.rng, 1.0))


class CubeLink:
    """One serial connection to the Cube, with a background reader thread.

    THE PORT ADMITS ONE READER, exactly like the camera. TELEM2 is a single
    UART: two processes opening it each receive a fraction of the bytes and both
    see CRC errors. If something else on this host already holds it, do not
    start a second link -- stop that first.
    """

    def __init__(self, device=None, baud=None, source_system=None,
                 attitude_hz=None, cue_hz=None, keep=None, want_send=True,
                 rc_hz=None):
        self.device = str(config.CUBE_DEVICE if device is None else device)
        self.baud = int(config.CUBE_BAUD if baud is None else baud)
        self.source_system = int(config.CUBE_SOURCE_SYSTEM
                                 if source_system is None else source_system)
        self.attitude_hz = float(config.CUBE_ATTITUDE_HZ
                                 if attitude_hz is None else attitude_hz)
        self.cue_hz = float(config.CUBE_CUE_HZ if cue_hz is None else cue_hz)
        # RC_CHANNELS drives the arming switches. Requested, not configured: this is
        # SET_MESSAGE_INTERVAL, so nothing has to change on the flight controller --
        # no RCn_OPTION, no custom mode, no firmware change.
        self.rc_hz = float(config.CUBE_RC_HZ if rc_hz is None else rc_hz)
        self.keep = int(config.CUBE_QUAT_KEEP if keep is None else keep)
        self.want_send = bool(want_send)

        self.master = None
        self.sysid = self.compid = None
        self.can_send = False
        self.cue = TargetCue()

        # Mount pitch as the CUBE has it. Read, never assumed and never applied
        # here: it is applied on the Cube, and a mismatch against
        # config.CAM_PITCH_DEG means the two ends disagree about the airframe.
        self.cam_pitch_deg = None

        self._quats = []           # [(t_mono, np.array([w, x, y, z])), ...] ascending
        self._acks = {}            # COMMAND_ACK results parked by the reader thread;
                                   # per instance, never a class-level mutable default
        self.params = {}           # every PARAM_VALUE seen, by name
        self.rc = {}               # chan -> (pulse_us, t_mono of the packet)
        self.nvf = {}              # NAMED_VALUE_FLOAT by name. Expected to stay EMPTY on
                                   # TELEM2: broadcast float telemetry is suppressed on a
                                   # private channel, so this doubles as a check of
                                   # whether MAV3_OPTIONS really has NO_FORWARD set.
        self._lock = threading.Lock()
        self._slerp = None
        self._thread = None
        self._run = False
        self._seq = 0
        self.n_sent = 0
        self.n_attitude = 0
        self.armed = None
        self.t_heartbeat = None

    # ---- lifecycle -----------------------------------------------------------
    def start(self, timeout=None):
        """Connect, wait for a heartbeat, request the streams. Returns self."""
        ok, detail = dialect_check()
        if not ok and self.want_send:
            raise RuntimeError("dialect is unusable: %s" % detail)

        timeout = float(config.CUBE_HEARTBEAT_TIMEOUT_S if timeout is None else timeout)
        self.master = mavutil.mavlink_connection(
            self.device, baud=self.baud, source_system=self.source_system)

        hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        if hb is None:
            self.master.close()
            self.master = None
            raise TimeoutError(
                "no heartbeat on %s @ %d in %.0fs. Check: wiring (TX/RX must cross), "
                "the Cube is powered, SERIAL2_PROTOCOL=2, SERIAL2_BAUD=57, and that "
                "nothing else on this host holds the port."
                % (self.device, self.baud, timeout))
        self.sysid = self.master.target_system = hb.get_srcSystem()
        self.compid = self.master.target_component = hb.get_srcComponent()
        self._note_heartbeat(hb)

        self._run = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

        acks = {}
        acks["ATTITUDE_QUATERNION"] = self.request_interval(
            MAVLINK_MSG_ID_ATTITUDE_QUATERNION, self.attitude_hz)
        if self.cue_hz > 0:
            acks["GCS_TARGET_BEARING"] = self.request_interval(
                MAVLINK_MSG_ID_GCS_TARGET_BEARING, self.cue_hz)
        if self.rc_hz > 0:
            acks["RC_CHANNELS"] = self.request_interval(
                MAVLINK_MSG_ID_RC_CHANNELS, self.rc_hz)
        self.request_param(CAM_PITCH_PARAM)
        self.ack_results = acks

        self.can_send = self.want_send and hasattr(self.master.mav,
                                                   "detection_target_data_send")
        return self

    def close(self):
        self._run = False
        if self._thread is not None:
            self._thread.join(timeout=config.CUBE_THREAD_JOIN_S)
        if self.master is not None:
            self.master.close()
            self.master = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()

    # ---- requests ------------------------------------------------------------
    def request_interval(self, msg_id, hz):
        """SET_MESSAGE_INTERVAL. Returns the COMMAND_ACK result, or None if none came.

        param2 is MICROseconds. Watch the two traps: 0 does NOT mean "as fast as
        possible", it means "reset to default", and a message with no default
        (both of ours) is then OFF -- stop a stream with -1. And an interval
        under 3000 us is DENIED at SCHED_LOOP_RATE=400; the Cube explains that in
        a STATUSTEXT, which is blocked on this private channel, so the ACK is the
        only thing that tells you.
        """
        interval_us = -1 if hz is None or hz <= 0 else int(1e6 / hz)
        self.master.mav.command_long_send(
            self.sysid, self.compid, MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, interval_us, 0, 0, 0, 0, 0)
        return self._wait_ack(MAV.MAV_CMD_SET_MESSAGE_INTERVAL)

    def request_once(self, msg_id):
        """REQUEST_MESSAGE: exactly one message comes back."""
        self.master.mav.command_long_send(
            self.sysid, self.compid, MAV.MAV_CMD_REQUEST_MESSAGE, 0,
            msg_id, 0, 0, 0, 0, 0, 0)
        return self._wait_ack(MAV.MAV_CMD_REQUEST_MESSAGE)

    def request_param(self, name):
        self.master.mav.param_request_read_send(
            self.sysid, self.compid, name.encode("ascii"), -1)

    def get_param(self, name, timeout=None):
        """Request one parameter and wait for its PARAM_VALUE. None on timeout.

        Goes through the reader thread's `params` store, because THE READER
        THREAD OWNS recv AND NOTHING ELSE MAY CALL IT. Two readers on one serial
        port each get a fraction of the bytes; pyserial surfaces that as
        "device reports readiness to read but returned no data (device
        disconnected or multiple access on port?)", which reads like a hardware
        fault and is not one. Use this, never link.master.recv_match().
        """
        self.request_param(name)
        deadline = time.monotonic() + (config.CUBE_PARAM_TIMEOUT_S
                                       if timeout is None else timeout)
        while time.monotonic() < deadline:
            with self._lock:
                if name in self.params:
                    return self.params[name]
            time.sleep(0.02)
        return None

    def _wait_ack(self, cmd_id, timeout=None):
        """COMMAND_ACK for cmd_id. The reader thread owns recv, so it parks acks here."""
        deadline = time.monotonic() + (config.CUBE_ACK_TIMEOUT_S
                                       if timeout is None else timeout)
        while time.monotonic() < deadline:
            with self._lock:
                if cmd_id in self._acks:
                    return self._acks.pop(cmd_id)
            time.sleep(0.01)
        return None

    # ---- receive -------------------------------------------------------------
    def _rx_loop(self):
        while self._run:
            try:
                m = self.master.recv_match(blocking=True,
                                           timeout=config.CUBE_RX_TIMEOUT_S)
            except Exception:
                time.sleep(0.02)
                continue
            if m is None:
                continue
            t = m.get_type()
            if t == "ATTITUDE_QUATERNION":
                self._push(time.monotonic(), (m.q1, m.q2, m.q3, m.q4))
            elif t == "GCS_TARGET_BEARING":
                self.cue.update(m)
            elif t == "COMMAND_ACK":
                with self._lock:
                    self._acks[m.command] = m.result
            elif t == "PARAM_VALUE":
                pid = m.param_id
                if isinstance(pid, bytes):
                    pid = pid.decode("ascii", "ignore")
                pid = pid.strip("\x00")
                with self._lock:
                    self.params[pid] = float(m.param_value)
                if pid == CAM_PITCH_PARAM:
                    self.cam_pitch_deg = float(m.param_value)
            elif t in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
                now = time.monotonic()
                with self._lock:
                    for i in range(1, 19):
                        v = getattr(m, "chan%d_raw" % i, None)
                        # 0 and 65535 both mean "this channel is not present", and
                        # storing either as a pulse width would arm on a phantom.
                        if v and v != 65535:
                            self.rc[i] = (int(v), now)
            elif t == "NAMED_VALUE_FLOAT":
                name = m.name
                if isinstance(name, bytes):
                    name = name.decode("ascii", "ignore")
                with self._lock:
                    self.nvf[name.strip("\x00")] = float(m.value)
            elif t == "HEARTBEAT" and m.get_srcSystem() == self.sysid:
                self._note_heartbeat(m)

    def _note_heartbeat(self, hb):
        self.armed = bool(hb.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
        self.t_heartbeat = time.monotonic()

    def _push(self, t, q):
        q = np.asarray(q, dtype=float)
        n = np.linalg.norm(q)
        if n < 1e-9:
            return
        with self._lock:
            self._quats.append((t, q / n))
            if len(self._quats) > self.keep:
                del self._quats[:len(self._quats) - self.keep]
            self.n_attitude += 1

    # ---- attitude lookup -----------------------------------------------------
    def q_at(self, t_mono):
        """Attitude (w, x, y, z) at t_mono, SLERPed between bracketing samples.

        body -> NED, Hamilton, applied FORWARD (v_ned = q (x) v_body (x) q*).
        ArduPilot's own header comments claim the opposite; the code does not,
        and RPI_COMMS.md section 4.1 has the citation. Returns None while the
        ring is empty, and clamps to the ends rather than extrapolating: a
        clamped value is a known small error, an extrapolated one is unbounded.
        """
        with self._lock:
            n = len(self._quats)
            if n == 0:
                return None
            if n == 1:
                return tuple(self._quats[0][1])
            ts = [s[0] for s in self._quats]
            samples = list(self._quats)
        if t_mono <= ts[0]:
            return tuple(samples[0][1])
        if t_mono >= ts[-1]:
            return tuple(samples[-1][1])
        j = int(np.searchsorted(ts, t_mono))
        t0, t1 = ts[j - 1], ts[j]
        frac = 0.0 if t1 <= t0 else (t_mono - t0) / (t1 - t0)
        if self._slerp is None:
            self._slerp = _load_slerp()
        return tuple(self._slerp(samples[j - 1][1], samples[j][1], frac))

    def attitude_span(self):
        """(oldest, newest, count) monotonic stamps in the ring -- is the lookup covered?"""
        with self._lock:
            if not self._quats:
                return None, None, 0
            return self._quats[0][0], self._quats[-1][0], len(self._quats)

    def attitude_hz_measured(self):
        with self._lock:
            if len(self._quats) < 2:
                return 0.0
            dt = self._quats[-1][0] - self._quats[0][0]
            return (len(self._quats) - 1) / dt if dt > 0 else 0.0

    def rc_us(self, chan):
        """(pulse_us, age_s) for one RC channel, or (None, None) if never seen.

        AGE IS RETURNED, NOT HIDDEN. A caller deciding whether to arm has to know
        how old the switch reading is: holding the last value after the link drops
        is exactly the failure an arming switch must not have.
        """
        with self._lock:
            v = self.rc.get(int(chan))
        if v is None:
            return None, None
        return v[0], time.monotonic() - v[1]

    # ---- send ----------------------------------------------------------------
    def send_detection(self, az_rad, el_rad, valid, confidence=1.0,
                       capture_usec=None, size_rad=0.0, los_ned=None,
                       capture_latency_us=0):
        """One DETECTION_TARGET_DATA per detector frame.

        SEND valid=0 FRAMES TOO. Silence for 0.5 s (LAT_DET_TIMEOUT_S) makes
        guidance treat detection as stale and hold a_des = 0; a valid=0 frame
        marks it inactive the moment it arrives. Silence is a timeout, valid=0
        is information.

        az/el must be CAMERA frame with mount pitch NOT applied -- use
        comms.bearing.CamModel.az_el(), which is built for exactly this.

        los_ned, if given, is the (n, e, d) unit LOS with the camera mounting AND
        the vehicle attitude fully applied -- comms.los.LosSolver.ned(). It rides
        in fields that already exist in this 52-byte message and are otherwise
        sent as zeros, so including it costs nothing on the wire.

        THE FIRMWARE IGNORES IT TODAY. RPI_COMMS.md section 3 is explicit that
        los_n/e/d are decoded and discarded because the frame=1 path is not wired
        up, and that `frame` must stay 0. Guidance steers on bearing_az/el. This
        is sent for the record, for the Cube-side log to be checkable against,
        and for whenever that path is finished -- not because anything acts on it.

        capture_latency_us: how old bearing_az/el actually were at send time --
        this detection's own row-dependent capture latency plus this frame's
        processing time (tools/flight_pipeline.py computes it per detection, see
        row_capture_latency_s()). A MAVLink 2 EXTENSION field (RPI_COMMS.md
        section 11): excluded from CRC_EXTRA, appended after the 41-byte base
        payload, so a Cube build that does not know about it decodes the same
        message it always has and just ignores the extra 4 bytes -- no
        coordination with the firmware side required to keep sending this.
        """
        if not self.can_send:
            return False
        self._seq = (self._seq + 1) & 0xFFFF
        self.master.mav.detection_target_data_send(
            int(capture_usec) if capture_usec is not None else 0,
            float(az_rad), float(el_rad),
            # los_n/e/d: the full-mount, full-attitude LOS. Decoded but ignored by
            # the firmware (frame=1 is not wired up); sent because the fields are
            # already there and a zero is strictly less useful than the truth.
            float(los_ned[0]) if los_ned is not None else 0.0,
            float(los_ned[1]) if los_ned is not None else 0.0,
            float(los_ned[2]) if los_ned is not None else 0.0,
            float(confidence),        # decoded by the firmware, then ignored
            float(size_rad),
            self._seq,
            0,                        # frame: ALWAYS 0. Setting 1 would not switch
                                      # the firmware to los_n/e/d -- that path does
                                      # not exist -- and `frame` is ignored anyway.
            1 if valid else 0,        # the field guidance actually reads
            0,                        # target_id
            int(capture_latency_us))  # EXTENSION FIELD, see docstring above
        self.n_sent += 1
        return True

    # ---- reporting -----------------------------------------------------------
    def status(self):
        oldest, newest, n = self.attitude_span()
        now = time.monotonic()
        return {
            "device": "%s@%d" % (self.device, self.baud),
            "sysid": self.sysid,
            "armed": self.armed,
            "heartbeat_age_s": None if self.t_heartbeat is None else now - self.t_heartbeat,
            "attitude_n": n,
            "attitude_hz": round(self.attitude_hz_measured(), 2),
            "attitude_age_s": None if newest is None else round(now - newest, 3),
            "attitude_span_s": None if n < 2 else round(newest - oldest, 1),
            "cam_pitch_deg": self.cam_pitch_deg,
            "rc": {c: v[0] for c, v in sorted(self.rc.items())},
            "cue_valid": self.cue.valid,
            "cue_n": (self.cue.n_valid, self.cue.n_invalid),
            "can_send": self.can_send,
            "n_sent": self.n_sent,
        }
