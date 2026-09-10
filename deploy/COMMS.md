# Bringing up the Cube link

`RPI_COMMS.md` (repo root) is the protocol contract — what the messages are and
what the firmware does with them. This file is the operational half: how to make
the link work on *this* Pi, and how to tell which thing is broken when it doesn't.

```bash
bash deploy/setup_comms.sh --check     # report, change nothing
bash deploy/setup_comms.sh             # set up, then run the checklist
python3 tools/cube_probe.py            # the checklist on its own, any time
```

## What the link carries

| direction | message | rate | why |
|---|---|---|---|
| Cube → Pi | `ATTITUDE_QUATERNION` (31) | 30 Hz | the live attitude that lets the LOS tracker run on a live feed at all |
| Cube → Pi | `GCS_TARGET_BEARING` (42051) | 5 Hz, on request | the radar cue, already in camera frame — a search prior |
| Pi → Cube | `DETECTION_TARGET_DATA` (42050) | one per detector frame | camera-frame bearing + validity |

Nothing streams until asked. `MAV3_POSITION` and `MAV3_EXTRA1` are both 0 and
`ATTITUDE_QUATERNION` is in no stream group at all, so no Cube parameter can
enable it — only `SET_MESSAGE_INTERVAL`. A connection showing nothing but
heartbeats is correct, not broken.

## Wiring — 3 wires, and one of them must not be connected

| TELEM2 pin | signal | Pi header pin | Pi function |
|---|---|---|---|
| 1 | VCC 5 V | **leave disconnected** | — |
| 2 | TX (Cube out) | 10 | GPIO15 / UART RXD |
| 3 | RX (Cube in) | 8 | GPIO14 / UART TXD |
| 6 | GND | 6 | GND |

TX and RX must cross. Both ends are 3.3 V TTL, so no level shifter — and the Pi
is not 5 V tolerant, which is why TELEM2 pin 1 stays disconnected: tying two 5 V
rails together back-feeds the weaker one.

## The four things that fail silently

In order of how long each costs to find:

1. **`dtparam=uart0=on`.** On a Pi 5, `enable_uart=1` covers only the dedicated
   debug connector. Without `uart0=on`, GPIO14/15 are not a UART and the wiring
   is irrelevant. Needs a reboot. *(Set on this host.)*
2. **The pymavlink dialect.** 42050 and 42051 are this airframe's own messages.
   Stock pymavlink raises `AttributeError` on the first send — and **drops an
   incoming 42051 without a word**, which is indistinguishable from firmware
   that does not implement it. `comms/gen_dialect.py` builds a dialect called
   `thermal_link` beside the stock one, so a `pip install --upgrade pymavlink`
   cannot silently revert the link, and so the other MAVLink consumer on this
   host keeps its own dialect untouched.
3. **`MAVLINK20=1` before pymavlink is imported.** The v1.0 dialect's message-id
   field is 8 bits and cannot represent an id above 255 at all. `comms/dialect.py`
   is the only supported way to reach `mavutil` in this repo for exactly this
   reason — it raises if pymavlink was already imported, because by then the
   environment variables are too late to matter.
4. **One reader per port.** TELEM2 is a single UART. Two processes reading it
   each get a fraction of the bytes; pyserial surfaces that as *"device reports
   readiness to read but returned no data (device disconnected or multiple
   access on port?)"*, which reads like a hardware fault and is not one. Inside
   this repo, `CubeLink`'s reader thread owns `recv` — use `link.get_param()`,
   never `link.master.recv_match()`.

## Verified on this hardware, 2026-09-09

Cube disarmed, STANDBY, mode 0. Read live off the Cube, not assumed:

| | |
|---|---|
| `SERIAL2_BAUD` | **57** — confirms 57600, not the 115 an older param file claims |
| `SERIAL2_PROTOCOL` | 2 (MAVLink 2, required: ids above 255) |
| `MAV3_OPTIONS` | 2 — `NO_FORWARD`, i.e. TELEM2 really is a private channel |
| `MAV3_EXTRA1`, `MAV3_POSITION` | 0, 0 — nothing auto-streams |
| `LAT_CAM_PITCH` | −25.00°, matching `config.CAM_PITCH_DEG` |

- `ATTITUDE_QUATERNION` at **30.3 Hz** measured; ring spans 13.2 s, newest sample
  5–30 ms old; `q_at()` returns a unit quaternion (norm 1.000000000).
- `GCS_TARGET_BEARING` **decodes correctly** — one-shot and at 5 Hz, both
  `COMMAND_ACK` ACCEPTED, `stop (-1)` ACCEPTED. It came back with
  `gcs_target_valid=0`, which is the right answer on a bench with no radar and
  no GPS, and it proves the request and decode path before a target exists.
  **This also validates the reconstructed message definition**: an ACCEPTED ack
  with a decoded message means the CRC_EXTRA agrees with the firmware's.
- `DETECTION_TARGET_DATA` **CRC_EXTRA is 87**, identical to the dialect that has
  been sending 42050 to this Cube since 2026-08-08 — so that reconstruction is
  wire-compatible, independently of anything this repo did.
- 133 detections sent at **11.0 Hz** with the attitude stream unaffected
  (30.33 Hz throughout).
- Zero `NAMED_VALUE_FLOAT` received while sending, which independently confirms
  the private channel: there is **no feedback path**, so this side can never
  observe its own detections being accepted.

**Not verified, and it cannot be from here:** that the Cube actually decoded
those detections. Nothing acknowledges `DETECTION_TARGET_DATA`, and the private
channel blocks the float telemetry that would otherwise show it. Pull the Cube's
**`LATD`** dataflash message and confirm `DAz`/`DEl` match what was sent and
`Det = 1`. `tools/cube_probe.py --send-detection` prints the exact numbers to
compare against.

## The az/el convention — pitch on the Cube, roll on the Pi

Send the bearing in the camera's own frame. The Cube applies the mount **pitch**
from `LAT_CAM_PITCH`; if this side applied it too it would land twice.

The exception the protocol document does not cover: the firmware models mount
**pitch only**, so mount **roll** has to come off here or nothing ever removes
it — and this core is bolted on rolled 90°, which exchanges the two image axes.
`comms/bearing.py` removes the roll and never touches the pitch.
`config.CAM_ROLL_DEG` is +90, measured by correlating predicted against
phase-correlated observed image motion over 524 frame pairs (+90 gives +0.735,
−90 gives −0.735 — an exact negation, i.e. 180° about the boresight).

Checked against the flight-proven implementation on this host: bearings agree to
**8.8e-32 degrees** over eight test pixels, and image corner (0,0) maps to
az −18.66° / el +21.80°, the documented truth for focal 1516 px.

## The final LOS, and who applies the mount

Three frames, two rotations, and the split of responsibility that is easy to get
wrong twice:

```
sensor    X = boresight, Y = image-right, Z = image-down
  |  R_bc    the camera's MOUNTING   -- fixed, measured, config.MOUNT_R_BC
  v
body FRD  X = forward, Y = right, Z = down
  |  q       the vehicle's ATTITUDE  -- live, 30 Hz from the Cube
  v
NED       X = north, Y = east, Z = down
```

| quantity | mount applied | who computes it | who acts on it |
|---|---|---|---|
| `bearing_az` / `bearing_el` | **roll only** | `comms/bearing.py` | **the Cube** — guidance steers on this |
| `los_n` / `los_e` / `los_d` | **roll + pitch, in full** | `comms/los.py` | nobody, today |

The split exists because the firmware models mount **pitch** and applies
`LAT_CAM_PITCH` itself, but does not model mount **roll** at all. So roll must
come off on this side or nothing ever removes it, and pitch must *not*, or it
lands twice. The full LOS is the opposite case: a world-frame vector has no
second party to finish the job, so `comms/los.py` applies `R_bc` entire.

**The firmware ignores `los_n/e/d`.** `RPI_COMMS.md` section 3 is explicit: they
are decoded and discarded because the `frame=1` path is not wired up, and
`frame` must stay 0. They ride in fields that were already in the 52-byte
message being sent as zeros, so including them costs nothing on the wire — but
guidance is steering on `bearing_az`/`bearing_el` and nothing else. Do not fly
expecting otherwise.

What it is good for: the flight CSV carries `body_az_deg`, `body_el_deg` and
`los_n/e/d` per frame, so a sortie can be reconstructed in world coordinates and
checked against the Cube's own `LATD` LOS, which the Cube derives independently
from the bearing. Two derivations of the same direction that disagree is a
finding.

### Verified, 2026-09-10

- `comms/los.py` agrees with the flight-proven implementation on this host to
  **0.0e+00** across six test pixels, for both the sensor ray and the NED LOS
- `R_bc` is asserted to be a proper rotation at startup — a non-conformal mount
  matrix costs a silently skewed LOS rather than an error
- boresight maps to NED `d = -0.4226 = -sin(25 deg)`, i.e. the measured 25 degree
  up-tilt, and `|los| = 1.000000`
- over a simulated Cube with a pure-yaw attitude: NED elevation equalled body
  elevation and NED azimuth equalled body azimuth plus the yaw, which is the
  only thing a pure yaw may do
- **over the real serial link to the real Cube** (disarmed): 453 messages at
  25 fps, and at the vehicle's actual 38.5 degree yaw, body az -18.25 -> NED az
  20.2, which is body plus yaw to within rounding

## Sending detections is the one operation with consequences

`lat_intercept_note_detection()` decodes on any link in any flight mode, and in
`MODE_INTERCEPT` it steers. So:

- `tools/cube_probe.py` sends nothing unless given `--send-detection`, and
  refuses even then while the Cube reports **ARMED**.
- `THERMAL_UPLINK=0` runs the entire pipeline and transmits nothing — the right
  setting for a first flight-line check.
- Send `valid=0` frames rather than going quiet. Silence for 0.5 s
  (`LAT_DET_TIMEOUT_S`) makes guidance treat detection as stale and hold
  `a_des = 0`; a `valid=0` frame marks it inactive the moment it arrives.
  Silence is a timeout, `valid=0` is information.

## Rates, against the budget

57600 8N1 is 5760 usable bytes/s each way.

| | B/s | % |
|---|---|---|
| `ATTITUDE_QUATERNION` 30 Hz × 44 B | 1320 | 23 % |
| `DETECTION_TARGET_DATA` 11 Hz × 52 B | 572 | 10 % |
| `GCS_TARGET_BEARING` 5 Hz × 35 B | 175 | 3 % |

The uplink rate is set by how fast the detector runs, not by choice. At the
measured 6.6–11.1 fps it clears the 2 Hz staleness floor comfortably, but it is
below the 10–20 Hz that `RPI_COMMS.md` section 3.2 wants for good LOS-rate
quality — the Cube's LOS-rate finite difference uses the real measured `dt`, so
this degrades accuracy rather than breaking anything. `tools/live_track.py`
prints a warning if the achieved rate ever drops under 2 Hz.
