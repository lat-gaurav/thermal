# RPi ⇄ Cube comms — protocol reference for the perception link

**Audience:** whoever writes the RPi side (camera → detector → bearing) that talks to the CubeOrangePlus
flight controller over TELEM2.

**Status:** every claim below was re-derived from the firmware source for this revision. Where ArduPilot's
own comments or an earlier draft of this document disagreed with the code, the code wins and the discrepancy
is called out inline, because those are the places you would otherwise lose a day.

**Scope:** this is the whole contract. You should not need to read `mode_intercept.cpp` to build against it.

---

## 0. TL;DR

| | |
|---|---|
| Link | TELEM2 = `SERIAL2` = USART3, 3.3 V TTL UART, **57600 8N1**, MAVLink **2** |
| Cube system id | **1** (component 1). Pick anything else for yourself; 255 is the pymavlink default and is fine. |
| Nothing is auto-streamed | The Cube sends you heartbeats and nothing else until you ask. This is deliberate. |
| You send | `DETECTION_TARGET_DATA` (id **42050**), one per detector frame, event-driven |
| You request | `ATTITUDE_QUATERNION` (id 31) as a stream; `GCS_TARGET_BEARING` (id **42051**) one-shot or as a stream |
| Prerequisite | Regenerate pymavlink's dialect, and set `MAVLINK20=1` — see §6. Skip this and both custom messages vanish silently. |
| Params to change | **None.** |

---

## 1. The physical link

**Port.** TELEM2 on the CubeOrangePlus. Derived from the board's hwdef, not assumed:
`libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/hwdef.inc` declares

```
SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 UART7 OTG2
```

That list is 0-indexed onto ArduPilot's `SERIALn_` params, so `SERIAL0`=OTG1 (USB), `SERIAL1`=USART2
(TELEM1), **`SERIAL2`=USART3 (TELEM2)**. Every `SERIAL2_*` param below therefore IS the TELEM2 config.

> The same hwdef has pin comments reading `# USART3 serial3 telem2` — that "serial3" is 1-based counting,
> not the param number. `SERIAL_ORDER` is authoritative.

**Electrical.** 3.3 V TTL UART: TX, RX, GND, cross TX↔RX. RTS/CTS exist on the connector (`PD11 USART3_CTS`,
plus RTS) if you ever want hardware flow control — not needed at this traffic volume. RPi GPIO UART is 3.3 V
TTL, but confirm your specific wiring before powering up.

**Baud: 57600** (`SERIAL2_BAUD = 57`, units of 1000). **Not 115200.**

> An earlier draft of this document said 115200, and `simulator/sitl_runs/hw_start_params.parm` still says
> `SERIAL2_BAUD,115`. Both are wrong: that value was aspirational and has never been flashed — every flight
> log to date reads 57. The authoritative param file is `ardupilot_overlay/interception_params.txt`
> (line-by-line annotated) and its MP-loadable twin `interception_params_MP.param`, which both say 57.
> Setting the Cube to 115200 while your RPi is at 57600 kills the link with no error anywhere.

**Protocol: MAVLink 2** (`SERIAL2_PROTOCOL = 2`). Mandatory, not a preference — the custom message ids are
42050 and 42051, and MAVLink 1's message-id field is 8 bits, so it physically cannot carry an id above 255.

**Budget.** 57600 8N1 = 5760 usable bytes/s each way. Current use is ≈20 % up, ≈23 % down (§5).

---

## 2. Architecture — how the data actually moves

```
   ┌──────────────────────────┐
   │  Target drone  sysid 2   │       ┌─────────────────────────────┐
   │  (or the GCS radar feed) │       │   GCS  (Mission Planner +   │
   └────────────┬─────────────┘       │   custom visualiser)        │
                │ GLOBAL_POSITION_INT └──────────┬──────────────────┘
                │ 10 Hz, over the radio          │ TELEM1 / SERIAL1 / MAV2_*
                │                                │ chan 1 — NOT private
                ▼                                ▼
   ┌────────────────────────────────────────────────────────────────────────┐
   │                    CubeOrangePlus     sysid 1                          │
   │                                                                        │
   │   AP_Follow  ──►  s_gcs_target_pos   (target NED, HOME-relative)       │
   │                          │                                             │
   │                          ├──► MODE_INTERCEPT guidance                   │
   │                          │                                             │
   │                          └──► lat_get_gcs_target_bearing()             │
   │                                 NED ─► body ─► camera  (LAT_CAM_PITCH) │
   │                                        │                               │
   │   lat_intercept_note_detection()       │                               │
   │        ▲   camera ─► body ─► NED       │                               │
   │        │          (LAT_CAM_PITCH)      │                               │
   └────────┼───────────────────────────────┼───────────────────────────────┘
            │                               │   TELEM2 / SERIAL2 / MAV3_*
            │ DETECTION_TARGET_DATA         │   chan 2 — ★ PRIVATE ★
            │ id 42050, event-driven        │   57600 8N1, MAVLink 2
            │                               ▼
            │                        GCS_TARGET_BEARING  id 42051  (on request)
            │                        ATTITUDE_QUATERNION id 31     (on request)
            │                        HEARTBEAT           id 0      (automatic)
            │                               │
   ┌────────┴───────────────────────────────▼───────────────────────────────┐
   │                        RPi 5   (perception)                            │
   │      camera ─► detector ─► camera-frame bearing az/el ─► wire          │
   │      Python + pymavlink, MAVLINK20=1, regenerated dialect              │
   └────────────────────────────────────────────────────────────────────────┘
```

### 2.1 The two independent target estimates — do not conflate them

This is the single most important idea in the architecture. The Cube holds **two** estimates of the same
target, from two different sensors, and they are deliberately kept apart:

| | Source | Firmware state | Frame it lives in |
|---|---|---|---|
| **The cue** | GCS/radar, via `AP_Follow` | `s_gcs_target_pos`, `s_gcs_target_vel` | NED, HOME-relative |
| **The detection** | your camera | `s_los_ned`, `s_omega_ned` | NED unit vector + LOS rate |

Everything derived from the radar cue carries a `gcs_target_` prefix, in the firmware and on the wire, for
exactly this reason. `GCS_TARGET_BEARING` is **the cue**, converted into your frame for convenience. It is
*not* a detection, and feeding it back to the Cube as one would be a feedback loop — guidance already has
that cue first-hand, at full rate, with no round trip.

### 2.2 Channel numbering — why TELEM2 is `MAV3_*`

Stream-rate params are the **`MAV<n>_*`** family (`GOBJECT(_gcs, "MAV", GCS)` in `ArduCopter/Parameters.cpp`,
with per-channel subgroups named `"1"`, `"2"`, …). **They are not `SR<n>_*`** — that was the old name and an
earlier draft of this doc used it.

`n` is **1-based over MAVLink channels**, not over serial ports. On this vehicle:

| Port | Protocol | MAVLink chan | Param prefix | Role |
|---|---|---|---|---|
| `SERIAL0` OTG1 | MAVLink2 | 0 | `MAV1_` | USB |
| `SERIAL1` USART2 | MAVLink | 1 | `MAV2_` | TELEM1 → GCS radio |
| **`SERIAL2` USART3** | **MAVLink2** | **2** | **`MAV3_`** | **TELEM2 → your RPi** |
| `SERIAL5` UART7 | MAVLink2 | 3 | `MAV4_` | spare |

**This mapping shifts** if any earlier port stops being MAVLink — set `SERIAL1_PROTOCOL` to something else
and TELEM2 becomes chan 1 = `MAV2_*`, silently re-pointing every `MAV3_*` setting at the wrong link.

To verify on the vehicle non-destructively, read the dataflash **`MAV`** message: one row per channel with
`chan` (**0-based** — add 1 for the param digit), `packet_rx_success_count`, `flags`
(ACTIVE / STREAMING / PRIVATE / LOCKED), `stream_slowdown_ms` and `times_full`. TELEM2 is the row whose RX
count climbs at your detection rate, and the same row quantifies link saturation from the Cube's own
accounting rather than an estimate.

### 2.3 TELEM2 is a PRIVATE channel — what that means for you

`MAV3_OPTIONS = 2` sets bit 1, `Option::NO_FORWARD` (`GCS.h:542`), which calls `set_channel_private(chan)`
(`GCS_Common.cpp:195`). Consequences, all verified in source:

**Blocked on your link:**
- **Broadcast `NAMED_VALUE_FLOAT`.** `GCS::send_to_active_channels()` does `if (c.is_private()) continue;`
  (`GCS.cpp:310`).
- **`STATUSTEXT`.** `statustext_send_channel_mask()` does `ret &= ~private_channel_mask()` (`GCS.cpp:275`).
- **MAVLink forwarding**, both directions — the GCS cannot reach your RPi through the Cube, and the Cube
  stops *initiating* `TIMESYNC` on this channel.

**Not blocked:**
- **Requested streams.** `GCS_MAVLINK::update_send()` has no `is_private()` check at all. This is why
  `ATTITUDE_QUATERNION` and `GCS_TARGET_BEARING` work here.
- **Your uplink.** `MAVLink_routing::forward()` returns process-locally for private channels, and
  `DETECTION_TARGET_DATA` carries no `target_system`/`target_component`, so `get_targets()` leaves both at
  −1 = broadcast = match. Your detections reach guidance normally.

**Why it is set that way, so nobody "helpfully" clears it:** `mode_intercept.cpp` pushes **11**
`NAMED_VALUE_FLOAT`s (`los_n`, `los_e`, `los_d`, `det_ok`, `rpi_age`, `det_hz` in every mode; plus `rad_ok`,
`rho`, `tgt_n`, `tgt_e`, `tgt_d` while INTERCEPT is active) at `LAT_TLM_HZ`, plus periodic `STATUSTEXT`.
At 20 Hz that is 11 × 30 B × 20 = 6600 B/s against 5760 B/s of usable link — **over 100 % of your entire
downlink from that one source**, appearing the instant mode 29 is entered. A link measured in STABILIZE
looks perfectly healthy and tells you nothing. **Always measure in INTERCEPT.**

> ⚠ **`tgt_n` / `tgt_e` / `tgt_d` do NOT reach the RPi.** An earlier draft of this document said they arrive
> unconditionally at 20 Hz and invited you to build on them. They do not arrive at all. Use
> **`GCS_TARGET_BEARING`** (§4.2) — it carries the same cue, already in your camera frame, at a rate you
> control, with explicit age and validity. Those floats still reach the GCS, and all but `rad_ok`/`rho` are
> in the `LATG`/`LATP` dataflash messages, so nothing is lost for post-flight analysis.

---

## 3. Uplink — what you send: `DETECTION_TARGET_DATA`

Custom message, id **42050**, defined in `ardupilot_overlay/mavlink/detection_target_data.msg.xml`.

| Field | Type | Status |
|---|---|---|
| `bearing_az` | float, rad | **CONSUMED** — camera-frame azimuth |
| `bearing_el` | float, rad | **CONSUMED** — camera-frame elevation |
| `valid` | uint8 | **CONSUMED** — 1 = target seen this frame, 0 = not |
| `time_usec` | uint64, µs | decoded, **ignored** by the firmware (your own clock; keep sending it, it is useful in logs) |
| `los_n`, `los_e`, `los_d` | float | **ignored** — the `frame=1` path is not wired up. Send 0. |
| `confidence` | float 0..1 | **ignored**. Send your real value anyway; costs nothing. |
| `size_rad` | float, rad | **ignored** |
| `seq` | uint16 | **ignored** by the firmware — useful for your own gap detection |
| `frame` | uint8 | **ignored** — **always send 0** |
| `target_id` | uint8 | **ignored** — send 0 |

Confirmed by reading the handler, which is the entire receiving end:

```cpp
case MAVLINK_MSG_ID_DETECTION_TARGET_DATA: {
    mavlink_detection_target_data_t p;
    mavlink_msg_detection_target_data_decode(&msg, &p);
    lat_intercept_note_detection(p.bearing_az, p.bearing_el, p.valid != 0);
    break;
}
```

It is one `case` in `GCS_MAVLINK_Copter::handle_message`, so it decodes on **any** link and in **any** flight
mode. There is no sysid filter and no message signing.

Wire size: 41 B payload (40 after MAVLink 2 trailing-zero trim) + 12 B framing = **52 B**.

### 3.1 The az/el convention — CAMERA frame. Do not mount-correct.

**Send the bearing in the CAMERA's own frame, with zero mount offset.** The Cube applies the mount
correction. If you also apply it, it gets applied twice.

Build it straight from pixel + intrinsics, as if the camera's optical axis were the reference "forward":

```
e_cam = ( cos(el)·cos(az),  cos(el)·sin(az),  −sin(el) )
```

so the axes are x = camera boresight, y = right, z = down, **relative to the camera**, not to the airframe.
Equivalently: `az = atan2(y, x)`, `el = atan2(−z, hypot(x, y))`.

The firmware then rotates camera → body about the shared pitch (Y) axis using **`LAT_CAM_PITCH`** (degrees,
**+ve = boresight tilted DOWN** from body +X), before rotating body → NED with the vehicle attitude:

```
p = radians(LAT_CAM_PITCH)
e_body.x =  e_cam.x·cos(p) − e_cam.z·sin(p)
e_body.y =  e_cam.y
e_body.z =  e_cam.x·sin(p) + e_cam.z·cos(p)
```

- **Only pitch is modelled.** Roll/yaw mount offset is assumed zero, matching the planned forward-facing,
  downward-canted mount. If that changes, the firmware is where it gets extended — not your side.
- **`LAT_CAM_PITCH` default is −33** on this build. `0` makes the rotation a no-op (camera ≡ body).
- **Read it, don't hardcode it.** It is a normal ArduPilot param: `PARAM_REQUEST_READ` on connect. One
  number, configured once on the Cube, readable by any MAVLink client — no duplicated config on the RPi.
- The **same** param and the **same** rotation pair produce `GCS_TARGET_BEARING`, in the inverse direction.
  The firmware holds exactly one inverse pair (`lat_cam_to_body` / `lat_body_to_cam`) and both directions go
  through it, so the cue and your detection are guaranteed to be in the same frame and are directly
  comparable with no conversion.

`perception/bearing.py`'s `CamModel` does the pixel→bearing maths. Construct it with `mount_rpy_deg=(0,0,0)`
(the default) so it yields the camera-frame bearing this protocol wants.

### 3.2 Rate — event-driven, with one hard timeout

Send one message per detector output frame, at whatever rate that naturally is. The firmware assumes no
fixed period:

- **De-duplicated by timestamp** (`s_det_ms != s_det_proc_ms`), so a repeated message is never integrated
  twice, no matter how many internal callers read it.
- **The LOS-rate finite difference uses your actual measured `dt`**, not an assumed period.
- **`valid = 0` takes effect immediately.** The conversion is gated `if (!s_det_valid || …) return;` and
  liveness requires `s_det_valid`, so a `valid=0` frame marks detection inactive on arrival — you do not
  have to wait out a timeout to say "I lost it". Send `valid=0` frames; don't just go quiet.
- **0.5 s staleness timeout** (`LAT_DET_TIMEOUT_S`). Exceed it and the firmware treats detection as stale
  and guidance holds `a_des = 0`. So stay above ~2 Hz while tracking; **10–20 Hz+** for usable LOS-rate
  quality. Measured on the real detector: 21.9 msg/s median, 0 parse errors.
- **Smoothing is per-message, not per-unit-time**: the LOS-rate low-pass is `s_omega_ned = 0.5·old +
  0.5·new`. If your rate varies a lot, the *amount* of smoothing varies with it. Correctness is unaffected;
  just don't be surprised.
- **`LAT_DET_DLY_MS`** (default 0) lets the Cube de-rotate your bearing by the attitude it had
  `LAT_DET_DLY_MS` ago instead of the live attitude, from a 400 Hz / 128-sample attitude history. Set it to
  your measured capture-to-receipt latency if you know it. At 0 the live attitude is used.

---

## 4. Downlink — what you request

**Nothing streams to TELEM2 until you ask.** `MAV3_POSITION = 0` and `MAV3_EXTRA1 = 0`, and the channel is
private. If you connect and see only heartbeats, that is correct behaviour, not a fault.

| Message | id | How to get it | Wire size |
|---|---|---|---|
| `HEARTBEAT` | 0 | automatic, 1 Hz (not a stream, cannot be disabled) | 21 B |
| `ATTITUDE_QUATERNION` | 31 | `SET_MESSAGE_INTERVAL` only | 44 B |
| `GCS_TARGET_BEARING` | **42051** | `REQUEST_MESSAGE` **or** `SET_MESSAGE_INTERVAL` | 35 B |
| `PARAM_VALUE` | 22 | `PARAM_REQUEST_READ` — e.g. `LAT_CAM_PITCH` | — |
| `GLOBAL_POSITION_INT` | 33 | `SET_MESSAGE_INTERVAL` — **but you probably don't need it**, see §4.2 | 40 B |

### 4.1 `ATTITUDE_QUATERNION` — the attitude source to use

`q1, q2, q3, q4` = **(w, x, y, z)**, Hamilton convention, and it rotates **body → NED**: apply it forward
(`v_ned = q ⊗ v_body ⊗ q*`) to take a body vector into NED. `rollspeed`/`pitchspeed`/`yawspeed` are the body
gyro rates in rad/s. `repr_offset_q` is sent as all zeros and carries no information.

Wire size is **44 B**, not 60: the full payload is 48 B, but `repr_offset_q` is the trailing field and is all
zeros, so MAVLink 2's trailing-zero trim removes those 16 bytes → 32 B payload + 12 B framing. Worth knowing
before you sanity-check the link budget against a byte counter.

> **ArduPilot's own comments contradict this — the code does not.** `AP_NavEKF3.h:174` says `getQuaternion()`
> returns "the rotation from NED to XYZ (autopilot) axes", and `AP_NavEKF3_core.h:594` calls the underlying
> `outputDataNew.quat` a rotation "from local NED earth frame to body frame". Both are stale. The same
> quaternion, used with its **forward** rotation, is what `getRotationBodyToNED()` returns:
> ```cpp
> void NavEKF3_core::getRotationBodyToNED(Matrix3f &mat) const {
>     outputDataNew.quat.rotation_matrix(mat);   // <-- forward rotation == body->NED
> ```
> and the DCM and SIM backends fill the same field from a body→NED matrix. It is body→NED. Verified.

**Prefer it over `ATTITUDE`** (Euler `roll/pitch/yaw`): no gimbal-lock ambiguity, no Euler round-trip, and no
convention argument. Use `ATTITUDE` only as a human-readable cross-check.

**Why it must be requested:** `ATTITUDE_QUATERNION` belongs to **no stream group** — it appears only in
`mavlink_id_to_ap_message_id()`'s table and in `try_send_message()`. No `MAV<n>_*` param can ever enable it,
on any channel. (`ATTITUDE` *is* in `STREAM_EXTRA1`, but `MAV3_EXTRA1 = 0` turns that off.)

### 4.2 `GCS_TARGET_BEARING` — the radar cue, already in your camera frame

Custom message, id **42051**, defined in `ardupilot_overlay/mavlink/gcs_target_bearing.msg.xml`.

**What it is for.** The GCS/radar tells the Cube where the target is, as an NED position, via `AP_Follow` —
the same cue `MODE_INTERCEPT` guides on. This message re-expresses it as a **camera-frame az/el plus slant
range**, so you can point your search window at it, gate false positives, and pick a scale, without
carrying vehicle attitude or doing any frame maths.

| Field | Type | Meaning |
|---|---|---|
| `time_usec` | uint64, µs | Autopilot time-since-boot at which this bearing was **computed** (`AP_HAL::micros64()`). Use it to align against your own `ATTITUDE_QUATERNION` samples. |
| `gcs_target_az` | float, rad | Cued target azimuth, **CAMERA frame** |
| `gcs_target_el` | float, rad | Cued target elevation, **CAMERA frame** |
| `gcs_target_range` | float, m | Slant range, vehicle → cued target |
| `gcs_target_age_ms` | uint16, ms | Age of the underlying radar fix at computation time. When `valid=1` this is always **0…499** (see below). |
| `gcs_target_valid` | uint8 | 1 = usable. 0 = **do not use**; the four target fields are then all exactly zero. |

Wire size: 23 B payload + 12 B framing = **35 B**. At 5 Hz that is 175 B/s = **3 % of the link**.

**Same frame as your uplink, by construction.** `gcs_target_az/el` use the identical camera frame and the
identical `LAT_CAM_PITCH` correction as `DETECTION_TARGET_DATA.bearing_az/bearing_el`. `gcs_target_az −
bearing_az` is therefore a meaningful cue-vs-detection residual with no conversion.

**It is a hybrid, and that is the design.** The target *position* is `gcs_target_age_ms` old; the attitude
and own position are whatever is current at the moment of the call. So it answers *"where would the last
known target appear from where I am pointing right now"* — which is what a search prior wants. Nothing is
cached; it is recomputed per request.

**When `gcs_target_valid` reads 0.** Any of:
1. no radar cue has ever been decoded (`s_have_tgt` false);
2. the cue is stale — older than **`LAT_TGT_TIMEOUT_S` = 0.5 s**, the same deadman guidance itself obeys;
3. own position unknown (`get_relative_position_NED_home()` fails — needs home set **and** EKF *horizontal*
   position).

The message is still sent when invalid, deliberately: a requester that got nothing back could not tell "no
cue" from "message not supported" and would burn a timeout deciding.

> Because of (2), `gcs_target_age_ms` is bounded to **< 500 ms whenever `valid = 1`**. It tells you how far
> to widen the search window *within* that half-second, not whether the cue is minutes old — `valid` already
> answers that.

> On the bench with no GPS lock and no radar you will get `valid = 0` and zeros. That is the correct answer,
> and it is the ideal first test: it proves the request/ack/message path works before any target exists.
> Note (3) needs EKF **horizontal** position specifically — a GPS 3D fix and a readable `HOME_POSITION` are
> *not* sufficient, and in SITL horizontal position arrives ~20 s after "origin set".

**Use it as a prior, not a measurement.** Do not send it back as a detection.

---

## 5. Link budget

| Direction | Traffic | B/s | % of 5760 |
|---|---|---|---|
| Uplink | `DETECTION_TARGET_DATA` @ 22 Hz × 52 B | 1144 | **20 %** |
| Downlink | `ATTITUDE_QUATERNION` @ 30 Hz × 44 B | 1320 | **23 %** |
| Downlink | `GCS_TARGET_BEARING` @ 5 Hz × 35 B | 175 | **3 %** |
| Downlink | `HEARTBEAT` @ 1 Hz × 21 B | 21 | 0.4 % |

Measured on both flight logs so far: `stream_slowdown_ms = 0` and `times_full = 0` on chan 2 — no
saturation. 57600 is not a constraint at these rates; raise it only for a real need (a much higher
quaternion rate, say), and change the RPi in the same commit.

---

## 6. pymavlink setup — two gotchas that will cost you a day each

Both are already handled in this repo's `.venv`. On a fresh RPi environment you will hit both.

**1. Neither custom message exists in stock pymavlink.** `DETECTION_TARGET_DATA` (42050) and
`GCS_TARGET_BEARING` (42051) are ours. pymavlink has never heard of them until its dialect is regenerated
from `ardupilot_overlay/mavlink/*.msg.xml`, which `apply_overlay.sh` splices into ArduPilot's own
`ardupilotmega.xml`. Without regenerating:

- `m.mav.detection_target_data_send(...)` raises `AttributeError` — loudly, but only when that code path
  first fires;
- an incoming `GCS_TARGET_BEARING` decodes as an unknown message id and is **dropped silently**, which looks
  exactly like the firmware not supporting it.

Regenerate on the RPi with the same call `apply_overlay.sh` uses, pointed at a copy of the injected
`ardupilotmega.xml`:

```python
from pymavlink.generator import mavgen
import pymavlink
opts = mavgen.Opts(output=pymavlink.__path__[0] + "/dialects/v20/ardupilotmega.py",
                   wire_protocol="2.0", language="Python3", validate=False)
assert mavgen.mavgen(opts, ["/path/to/injected/ardupilotmega.xml"])
```

Verify it took:

```python
from pymavlink.dialects.v20 import ardupilotmega as d
print(d.MAVLink_gcs_target_bearing_message.id)          # -> 42051
print(d.MAVLink_detection_target_data_message.id)       # -> 42050
```

**2. Set `MAVLINK20=1` *before* importing pymavlink.**

```python
import os
os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil        # must come after
```

Without it pymavlink binds its v1.0 dialect module, which cannot encode a message id above 255 at all —
separately from the wire-format question.

If your RPi code is not Python: regenerate your dialect headers from the same `.msg.xml` files, and make
sure you encode as MAVLink 2.

---

## 7. How to take the data in — working code

Connect, and note that nothing arrives until you ask.

```python
import os, math
os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

MAV = mavutil.mavlink
GCS_TARGET_BEARING = 42051

m = mavutil.mavlink_connection("/dev/serial0", baud=57600)   # source_system defaults to 255
m.wait_heartbeat()
sysid, compid = m.target_system, m.target_component          # 1, 1
print(f"vehicle sysid {sysid}")
```

### 7.1 Read `LAT_CAM_PITCH` once, on connect

```python
def read_param(name, timeout=3.0):
    import time
    m.mav.param_request_read_send(sysid, compid, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if p and p.param_id.strip("\x00") == name:
            return p.param_value
    return None

cam_pitch_deg = read_param("LAT_CAM_PITCH")     # e.g. -33.0
```

You do not need it to *build* your bearing (you send camera-frame, uncorrected). Read it so you can
reproduce the Cube's frame maths for your own diagnostics, and so a mount change is visible to you.

### 7.2 Subscribe to attitude

```python
def set_interval(msg_id, hz):
    """Periodic subscription. interval is MICROseconds."""
    m.mav.command_long_send(sysid, compid, MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

def stop_stream(msg_id):
    m.mav.command_long_send(sysid, compid, MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msg_id, -1, 0, 0, 0, 0, 0)

def request_once(msg_id):
    """One-shot. Exactly one message comes back."""
    m.mav.command_long_send(sysid, compid, MAV.MAV_CMD_REQUEST_MESSAGE, 0,
                            msg_id, 0, 0, 0, 0, 0, 0)

def wait_ack(cmd_id, timeout=3.0):
    import time
    t0 = time.time()
    while time.time() - t0 < timeout:
        a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if a and a.command == cmd_id:
            return a.result          # 0 ACCEPTED, 2 DENIED, 4 FAILED
    return None

set_interval(MAV.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 30)
assert wait_ack(MAV.MAV_CMD_SET_MESSAGE_INTERVAL) == MAV.MAV_RESULT_ACCEPTED
```

**Re-request on every connection.** Message intervals are stored **per channel and in RAM only** — every
Cube reboot drops them. It is a *subscription*, not a poll: ask once, receive forever, until reboot.

> **Alternative that removes the dependency on your uplink working:** put a file
> `message-intervals-chan2.txt` on the Cube's SD-card root containing one `<mavlink_msg_id>
> <interval_MILLIseconds>` pair per line — **milliseconds here**, unlike `SET_MESSAGE_INTERVAL`'s
> microseconds. So:
> ```
> 31 33
> 42051 200
> ```
> gives you `ATTITUDE_QUATERNION` at ≈30 Hz and `GCS_TARGET_BEARING` at 5 Hz from boot, with no request at
> all. Read by `GCS_MAVLINK::initialise_message_intervals_from_config_files()`. The `chan` in the filename is
> the **0-based** channel (matching the dataflash `MAV` message's `chan`), *not* the `MAV3` digit. Worth
> doing: otherwise a dead TX line and a missing request look identical.
>
> One knock-on: an interval set this way *becomes* the message's default, so `param2 = 0` then means "reset
> to that file's rate" rather than "off" (`get_default_interval_for_ap_message()` consults the file before
> the stream groups). If you use the file, stop a stream with `-1`, never with `0`.

### 7.3 Take in `GCS_TARGET_BEARING`

Both access patterns work, through the same single registration. Pick per use case:

```python
# ONE-SHOT: on acquisition start, on a lost track, or whenever you want a fresh prior.
request_once(GCS_TARGET_BEARING)
assert wait_ack(MAV.MAV_CMD_REQUEST_MESSAGE) == MAV.MAV_RESULT_ACCEPTED
b = m.recv_match(type="GCS_TARGET_BEARING", blocking=True, timeout=2.0)

# PERIODIC: a background prior you can always read the latest of.
set_interval(GCS_TARGET_BEARING, 5)          # 5 Hz
assert wait_ack(MAV.MAV_CMD_SET_MESSAGE_INTERVAL) == MAV.MAV_RESULT_ACCEPTED

# STOP.
stop_stream(GCS_TARGET_BEARING)
```

**Interval rules, exactly as the firmware implements them** (`GCS_MAVLINK::set_message_interval`):

| `param2` (µs) | Effect |
|---|---|
| `-1` | stop sending |
| `< -1` | `MAV_RESULT_DENIED` |
| **`0`** | "reset to default rate" — and `GCS_TARGET_BEARING` **has no default**, so **0 means OFF**, not "as fast as possible" |
| `1 … 2999` | `MAV_RESULT_DENIED` at `SCHED_LOOP_RATE = 400` — see below |
| `3000 …` | accepted; interval = `param2 / 1000` ms |
| `> 60000000` | clamped to 60 s |

The floor comes from `cap_message_interval()`: a request is denied when
`interval_ms * 800 < get_loop_period_us()`. At `SCHED_LOOP_RATE = 400` (loop period 2500 µs) the **minimum
accepted interval is 3 ms (~333 Hz)**, and a denial also emits a `STATUSTEXT` reading *"Requested rate for
message ID … too fast. Increase SCHED_LOOP_RATE"* — which you will not see, because STATUSTEXT is blocked on
this private channel. Check the `COMMAND_ACK` result instead.

You will never go near the floor: **5–10 Hz is the right range**, because the underlying radar cue does not
update faster than that.

### 7.4 Consume it

```python
class TargetPrior:
    """Latest usable radar cue, in camera frame. Thread-free; just call update() with each message."""
    def __init__(self):
        self.az = self.el = self.rng = None
        self.age_ms = None
        self.t_usec = None

    def update(self, b):
        if b.gcs_target_valid != 1:
            # No usable cue. az/el/range are zero -- do NOT treat that as "target dead ahead".
            self.az = self.el = self.rng = None
            return False
        self.az, self.el = b.gcs_target_az, b.gcs_target_el
        self.rng, self.age_ms = b.gcs_target_range, b.gcs_target_age_ms
        self.t_usec = b.time_usec
        return True

    def search_window_rad(self, tgt_speed_mps=30.0, base_rad=math.radians(3.0)):
        """Widen the window for cue age and closeness. age_ms is < 500 when valid.
        A target moving tgt_speed_mps can have moved tgt_speed_mps * age_ms/1000 metres since the fix;
        at slant range rng that subtends roughly that over rng radians. Tune base_rad to your own
        radar/mount uncertainty -- it is not something the Cube can tell you."""
        if self.az is None:
            return None
        drift_m = tgt_speed_mps * (self.age_ms / 1000.0)
        return base_rad + (drift_m / max(self.rng, 1.0))

prior = TargetPrior()

while True:
    msg = m.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue
    t = msg.get_type()

    if t == "GCS_TARGET_BEARING":
        if prior.update(msg):
            half = prior.search_window_rad()
            # -> centre your detector's search window on (prior.az, prior.el) with half-width `half`,
            #    and size the expected blob from prior.rng.
        else:
            # -> search unaided. This is normal before GPS/EKF are ready or before the radar has a track.
            pass

    elif t == "ATTITUDE_QUATERNION":
        q = (msg.q1, msg.q2, msg.q3, msg.q4)     # (w, x, y, z), body -> NED, apply forward
        # keep a timestamped history if you want to pair attitude with image capture time

    elif t == "HEARTBEAT" and msg.get_srcSystem() == sysid:
        pass                                      # link liveness
```

### 7.5 Send your detections

```python
seq = 0
def send_detection(az_rad, el_rad, valid, confidence=1.0, capture_usec=None):
    """One call per detector frame. Send valid=0 frames too -- silence is a timeout, valid=0 is information."""
    global seq
    seq = (seq + 1) & 0xFFFF
    m.mav.detection_target_data_send(
        capture_usec if capture_usec is not None else 0,   # time_usec: your capture clock
        float(az_rad),        # bearing_az  -- CAMERA frame, no mount correction
        float(el_rad),        # bearing_el  -- CAMERA frame, no mount correction
        0.0, 0.0, 0.0,        # los_n/e/d   -- unused, frame=1 not wired
        float(confidence),    # confidence  -- decoded but unused today
        0.0,                  # size_rad    -- unused
        seq,                  # seq         -- your gap detection
        0,                    # frame       -- ALWAYS 0 (BODY/az-el path)
        1 if valid else 0,    # valid       -- READ by the firmware
        0)                    # target_id   -- 0
```

---

## 8. Bench-test checklist — do this before flying

Ground test, then SITL, then hardware. Bench (Cube + RPi on a desk) before flight.

1. Wire TELEM2 ↔ RPi UART. Confirm `SERIAL2_PROTOCOL = 2` and `SERIAL2_BAUD = 57` on the Cube, and 57600 on
   your side.
2. Confirm the dialect regeneration and `MAVLINK20=1` took, using the two `print()`s in §6. **Do this before
   anything else** — every later failure looks the same if you skip it.
3. Connect and confirm a `HEARTBEAT` from sysid 1. **Expect nothing else.** Silence beyond heartbeats is
   correct, not broken.
4. `PARAM_REQUEST_READ` `LAT_CAM_PITCH`; confirm you get a value (−33 by default).
5. Request `ATTITUDE_QUATERNION` at 30 Hz; confirm `COMMAND_ACK` result 0 and the measured rate. Tilt the
   Cube by hand and check the quaternion moves sensibly.
6. `REQUEST_MESSAGE` `GCS_TARGET_BEARING` (42051). Confirm `COMMAND_ACK` result 0 and **exactly one** message
   back. On a bench with no GPS and no radar, expect `gcs_target_valid = 0` and zeros — that is the correct
   "no cue" answer and it proves the whole path works before a target exists.
7. `SET_MESSAGE_INTERVAL` 42051 at 5 Hz; confirm the rate. Then `-1`; confirm it stops.
8. Send a `DETECTION_TARGET_DATA` with `valid=1` and a known `bearing_az/el`. Pull the Cube's dataflash
   **`LATD`** message and check `DAz`/`DEl` equal what you sent and `Det = 1`. That confirms the link, the
   message, and the handler. (`LATD` also carries `LosN/LosE/LosD`, the NED unit LOS derived from your
   bearing, and `AgeUs`/`AttOk` for the `LAT_DET_DLY_MS` path.)
9. Only then a flight test with INTERCEPT armed — and **measure the link in INTERCEPT**, never only in
   STABILIZE (§2.3).

---

## 9. Reference implementation

`isaac_bridge/twin_side/mavlink_feeder.py` plays the RPi's role in SITL against this same firmware and
message set, over a UDP loopback instead of a UART. Read it for the request/parse/send mechanics — its
`_request()` helper is the pattern to copy.

**Two things it does that you should NOT copy**, because its loopback is not a private channel and so
receives traffic TELEM2 never will:

- it reads `tgt_n/e/d` off `NAMED_VALUE_FLOAT` — those never arrive on TELEM2 (§2.3). Use
  `GCS_TARGET_BEARING`.
- its `det_range` acquisition gate computes range from `GLOBAL_POSITION_INT` + `tgt_n/e/d`.
  `gcs_target_range` gives you that number directly, which also removes any need for
  `GLOBAL_POSITION_INT`.

Porting to hardware is otherwise mechanical: swap the UDP transport for
`mavutil.mavlink_connection('/dev/serial0', baud=57600)`, and swap the Isaac scene for your real camera
capture plus `perception/` detector call.

`perception/` (repo root) is the detector, written with **zero Isaac/MAVLink dependencies** (numpy/scipy
only) precisely so it runs unchanged on hardware. `perception/bearing.py`'s `CamModel` turns pixel +
intrinsics into the `(az, el)` pair this protocol wants — see §3.1 for the `mount_rpy_deg=(0,0,0)` caveat.

---

## 10. Firmware / parameter provenance

- **Firmware:** ArduCopter 4.7-dev + LAT overlay, CubeOrangePlus. The build carrying `GCS_TARGET_BEARING` is
  at `Ardu_Codes/09Sep_GCS_TARGET_BEARING/arducopter.apj` (see its `README.txt`).
- **No parameter changes are needed** to use anything in this document. `GCS_TARGET_BEARING` is gated by no
  param — it is one `ap_message` enum value, one `id_map` row and one `try_send_message` case.
- **Authoritative params:** `ardupilot_overlay/interception_params.txt` (annotated) and
  `ardupilot_overlay/interception_params_MP.param` (Mission Planner loadable). Treat
  `simulator/sitl_runs/hw_start_params.parm` as stale for `SERIAL2_BAUD` and the `SR2_*` lines.
- **Message definitions:** `ardupilot_overlay/mavlink/detection_target_data.msg.xml`,
  `ardupilot_overlay/mavlink/gcs_target_bearing.msg.xml`.
- **Tests** (all passing on this build): `simulator/sitl_runs/test_gcs_target_bearing.py` (27/27 — request
  patterns, rate limits, geometry at `LAT_CAM_PITCH` 0 and −33, staleness),
  `simulator/sitl_runs/test_detection_regression.py` (12/12 — cue→bearing→detection→NED LOS round-trips to
  0.011°, log schemas unchanged), `simulator/sitl_runs/run_frame_roundtrip.sh` (the camera↔body pair are
  exact inverses).
- **Wider context:** `docs/CommsArchitecture.md` covers both links, including the Cube⇄GCS side.
