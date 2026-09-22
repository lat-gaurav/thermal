# Repository guide — `thermal`, all branches

**GitHub:** [github.com/lat-gaurav/thermal](https://github.com/lat-gaurav/thermal)

This is the map of the repo: what each of the three branches is *for*, how they
relate to each other, and a directory-by-directory tour of what lives where.
It exists because no single document ties `detector/`, `filters/`,
`initialisation/`, `experiment/`, `sot/`, `comms/`, `flight/`, `tools/` and
`deploy/` together — each has its own excellent docstring, but nothing says how
they compose into a running pipeline, or which of the three branches a given
file actually lives on.

It does **not** re-derive material that is already documented in depth
elsewhere in the repo; it points at that material and adds the connective
tissue. The two other long-form documents are:

- **[README.md](../README.md)** — the `main` branch's own subject: the
  latency-measurement rig and its C tools.
- **[RPI_COMMS.md](../RPI_COMMS.md)** — the MAVLink protocol contract with the
  CubeOrangePlus flight controller.

Every constant referenced below lives in **[config.py](../config.py)**, which
is the single source of truth for tunables and is itself heavily commented —
this guide does not duplicate those comments, only points at the section.

---

## 1. Three branches, one airframe project, two hardware generations

```
                                                      ┌─ 4957050 web: PMIC voltage/rail draw
                                        ┌─ 717f759 ───┤
                              b75ad13 ──┤             └─ (merged)
                                        │
  d10ded2 (main) ── 4d989f0 ── b53bb0b ─┴─ 5249121 ── d816a3f ─┬─ 28f9eec (flight-pipeline, HEAD)
   "latency rig"    "flight     "capture   "fail on    "manual  │
    (Jetson)         pipeline"   latency,   stale        click"  │
                                  SOT"       dialects"            │
                                                                  └─ 06c7843 (thermal-live-deploy)
                                                                     "strip to only what
                                                                      thermal-live.service needs"
```

| Branch | Commits | Hardware | What it is |
|---|---|---|---|
| **[`main`](https://github.com/lat-gaurav/thermal/tree/main)** | 10 | Jetson Orin Nano | The **latency-measurement rig**: C-only acquisition, blob logging, and the IMU-referenced latency estimator (`det_latency`). No MAVLink, no tracker, no flight logic. This is where the headline `~147 ms world→cx,cy` number and the whole "how do you measure latency without a clock that can see the world" methodology live. |
| **[`flight-pipeline`](https://github.com/lat-gaurav/thermal/tree/flight-pipeline)** *(current HEAD)* | 18 (10 shared + 8 new) | Raspberry Pi 5 | Branches off `main` at `d10ded2` and **adds** everything needed to actually fly: the Cube (ArduPilot) MAVLink link (`comms/`), a real detector/filter/tracker chain (`detector/`, `filters/`, `initialisation/`, `experiment/`), an operational flight service (`flight/`, `tools/flight_pipeline.py`), and a systemd deployment (`deploy/`). It keeps every `main`-branch tool too — see §6. |
| **[`thermal-live-deploy`](https://github.com/lat-gaurav/thermal/tree/thermal-live-deploy)** | 15 (14 shared with `flight-pipeline` + 1 new) | Raspberry Pi 5 | Branches off `flight-pipeline` at `d816a3f` and **removes** everything `thermal-live.service` cannot reach: the whole Jetson-era latency-rig toolchain (`bin/`, `calibration/`, `tools/blob_log.c`, `tools/imu_live.c`, `tools/record_raw.c`, `tools/view_camera.c`, `read_core.py`, `rawrec_view.py`, `rawrec_3d.py`, `rawrec_annotate.py`, `thermal_detect.py`, `dets.csv`, `tools/set_fps_jetson.py`, `tools/det_bench.py` — the last because it hard-imports `thermal_detect`/`rawrec_view`). One commit, `06c7843`, does this — see its message for the exact rationale per file. It has **not** received `flight-pipeline`'s three most recent commits (the SOT tracker, cached indexing, telemetry export, PMIC voltage, and the `cube_text` crash fix), so it is a point-in-time deployment snapshot, not a continuously-tracked deployment branch. |

The project is really one continuous story: a camera+detector latency rig on a
Jetson bench (`main`) grew, on a different host (an RPi5 riding a gimbal in the
air), into a full detect→track→uplink seeker (`flight-pipeline`), which was
then pruned down to a minimal deployable checkout (`thermal-live-deploy`) for
the one systemd unit that actually needs to run unattended. Nothing on
`thermal-live-deploy` is independent work — it is provably a subset of
`flight-pipeline`, verified file-by-file in the branch's own merge commit.

Working out of `flight-pipeline` (this branch) gives you the union of
everything: the Jetson latency tools **and** the RPi5 flight pipeline both
build and run from this one checkout.

---

## 2. Hardware, across both generations

| | Jetson rig (`main`) | RPi5 rig (`flight-pipeline`, `thermal-live-deploy`) |
|---|---|---|
| Host | Jetson Orin Nano, p3768 carrier, L4T R36.4.7 | Raspberry Pi 5 |
| Camera | Artosyn "Sirius" thermal core, `1d6b:0101`, 1280×1024 GREY @ 25 fps over USB 2.0 (same core, same limits, on both hosts) | same |
| Sensor | 640×512 — the 1280×1024 stream is a 2× upscale (three quarters of every frame is interpolated) | same |
| Lens | 27.3 px/deg (46.8° HFOV), measured | same |
| Extra sensor | Gimbal telemetry over FTDI FT232R, 921600 baud, ~1010 Hz (used as the *external clock* for latency measurement — see README.md) | *(not used on this rig)* |
| Flight controller | — (bench rig, no autopilot) | CubeOrangePlus, TELEM2 = `SERIAL2` = USART3, 3.3 V TTL, **57600 8N1**, MAVLink 2 — see RPI_COMMS.md |
| Mount | rigid, on a test gimbal | rigid, forward-facing on the airframe, **rolled 90° about the boresight and pitched 25° up** — see `config.CAM_ROLL_DEG` / `config.CAM_PITCH_DEG` / `config.MOUNT_R_BC`. This means, in the raw frame, **earth sits on the left and sky on the right** rather than the usual top/bottom split. |

The 90°-roll mount fact matters for reading raw footage from either rig and is
why `initialisation/rightmost_isolated.py` picks the *rightmost* isolated
detection (sky, where a distant target is expected to be alone) rather than
the topmost.

---

## 3. `main` — the latency-measurement rig (Jetson)

Fully documented in **[README.md](../README.md)**; this section is only an
index into it plus the file list, so this guide stays a complete map even for
files that are described in depth elsewhere.

**Build** (no build system — each tool is one translation unit):

```bash
mkdir -p bin
gcc -O2 -Wall -Wextra -o bin/record_raw   tools/record_raw.c
gcc -O2 -Wall -Wextra -o bin/blob_log     tools/blob_log.c     -lpthread -ljpeg -lm
gcc -O2 -Wall -Wextra -o bin/det_latency  calibration/det_latency.c  -lm
gcc -O2 -Wall -Wextra -o bin/imu_live     tools/imu_live.c     -lm
gcc -O2 -Wall -Wextra -o bin/view_camera  tools/view_camera.c  -lpthread -ljpeg
gcc -O2 -Wall -Wextra -o bin/detect_blob  calibration/detect_blob.c  -lm
gcc -O2 -Wall -Wextra -o bin/flow_stamp   calibration/flow_stamp.c   -lpthread -lm
gcc -O2 -Wall -Wextra -o bin/jerk_latency calibration/jerk_latency.c -lm
```

| File | Role |
|---|---|
| `tools/blob_log.c` | **The main recorder.** Logs IMU at ~1 kHz and a blob detection per frame, with a droppable live preview, under a strict `frames > IMU > detection > preview` priority order enforced structurally (every branch out of the acquisition thread ends in `QBUF`). |
| `tools/record_raw.c` | Every raw sensor byte to disk, no compression, no detection — raw V4L2/`ioctl()` only, deliberately not linked against `libv4l2` (which silently converts pixel formats). |
| `tools/imu_live.c` | Live yaw/pitch/rate with a px/frame verdict on whether the current pan speed is trackable. **Never run alongside `blob_log`** — both open the same tty and steal each other's bytes (measured: 1009→780 Hz, 182 CRC errors). |
| `tools/view_camera.c` | MJPEG preview over HTTP, no detection, structured so a slow network client can never cost a camera frame. |
| `calibration/det_latency.c` | **The latency measurement itself.** IMU as an external clock; sweeps a time-shift τ and fits a joint 2×2 rotation+scale map between image velocity and gimbal rate; the τ minimising the residual is the latency. Refuses rather than guessing when the fit isn't a plausible rigid rotation. Validated against synthetic data to within 1.6 ms. |
| `calibration/detect_blob.c` | Offline blob detection + oscillation analysis (DFT) over a finished `.gray` capture. |
| `calibration/flow_stamp.c` | Per-frame optical flow by 1-D profile alignment; dual-timestamped. Superseded by `blob_log` for latency work but kept as the reference for frame timing. |
| `calibration/jerk_latency.c` | Onset-pairing latency estimator. **Superseded, not recommended** — see README §"What did not work": fast jerks make the tracker confidently lock onto the wrong feature, which is the opposite of the assumption the method needs. |
| `calibration/check_gimbal.py` | Six-stage gimbal telemetry link diagnosis (USB present → port opens → bytes arrive → frames parse → CRC → values actually change). Receive-only. |
| `read_core.py` | Reads every documented read-only register/status value off the thermal core's serial control channel (`--selftest`, `--json`). Read-only, safe to run against a live stream. |
| `tools/set_fps.py` | Original RPi script for the core's 25/50 fps control code. **Do not edit** — hardcodes `/dev/thermal0`, which does not exist on the Jetson. |
| `tools/set_fps_jetson.py` | Jetson-portable fork of the above: resolves the capture node via the stock `/dev/v4l/by-id/...` symlink instead of the RPi's custom udev rule, and distinguishes the core's two v4l2 nodes (capture vs. metadata). |
| `rawrec_view.py`, `rawrec_3d.py`, `rawrec_annotate.py` | Frame-by-frame `.rawrec` viewer (with a live small-target detector and derivative-profile panel), a 3-D surface view of one frame, and a ground-truth labelling/scoring tool. |
| `thermal_detect.py` | A from-scratch small-target detector trained/validated against 659 hand-labelled targets (contrast + isolation "moat" + local crowd count). 97.0% recall / 0.065 FA-per-frame held out. |
| `docs/DEVICE_NODES.md` | Full USB topology; why `videoN`/`ttyACMn`/`ttyUSBn` numbers drift and how every tool resolves nodes by USB serial instead. |
| `docs/THERMAL_SERIAL_FAULT.md` | Root-cause writeup of a four-day serial-channel outage caused by ModemManager AT-probing the core's CDC-ACM port, and the udev rule (`99-thermal-core-mm-ignore.rules`) that fixes it. |
| `docs/important_comands.txt` | Working command cheat-sheet: network setup for the Jetson-over-Ethernet bench setup, and the two recording rules that make `det_latency` runs usable (sweep < ~33°, peak rate 30–40°/s). |

**Output formats** (used by every reader across all three branches — see §7
for the full field reference): `blobs.csv` (28 named columns, one row per
acquired frame), `imu.csv` (one row per telemetry frame that passed CRC), and
the `.rawrec` binary capture format.

---

## 4. `flight-pipeline` — the operational seeker (current HEAD)

This is the branch this guide lives on. Everything in §3 above still builds
and runs here unchanged; this section covers what was added on top of it to
turn a bench latency-measurement rig into something that flies.

### 4.1 What's new, in one paragraph

The old service's detection/tracking is replaced with this repo's own
detector (`detector/`) gated by clutter filters (`filters/`), acquired by a
choice of strategies (`initialisation/`), driven by an attitude-aware tracker
(`experiment/los_static_track.py`'s `SmoothTracker`), fed a *live* camera
attitude at 30 Hz from the CubeOrangePlus over MAVLink (`comms/`), wrapped in
the same operational envelope (two independent RC switches, `.rawrec`
episodes, fail-safe disarm) the rig already flew under (`flight/`,
`tools/flight_pipeline.py`), and installed as a systemd service
(`deploy/`). A single-object appearance tracker (`sot/`) is available as an
independent second opinion, wired into the telemetry-replay viewer.

### 4.2 `config.py` — read this first

Every tunable in the repo lives in one 602-line file, organised into 12
numbered sections, each naming which module reads it:

| § | Section | Consumers |
|---|---|---|
| 1 | The camera | `flight/camera.py`, `tools/*` |
| 2 | Detectors | `detector/tophat_scr.py`, `detector/boxtophat_scr.py`, `thermal_detect.py` |
| 3 | Filters | `filters/clutter_reject.py`, `filters/los_proximity.py` |
| 4 | Acquisition | `initialisation/*` |
| 5 | Tracker | `experiment/los_static_track.py`'s `SmoothTracker` |
| 6 | Camera model & mount | `comms/bearing.py`, `comms/los.py` |
| 7 | Timing: pairing a frame with an attitude | `comms/cube_link.py`, viewers |
| 8 | The Cube link | `comms/` |
| 9 | The RC switches | `flight/rc_arm.py` |
| 10 | Recording | `flight/rawrec.py` |
| 11 | The flight pipeline | `tools/flight_pipeline.py` |
| 12 | Viewers and tools | `tools/rawrec_viewer.py`, `web_viewer.py`, `fb_viewer.py`, `telemetry_viewer.py`, `preview.py` |

The file's own docstring states what is **deliberately not** here: wire
formats (`.rawrec`'s layout, MAVLink message ids), measured physical constants
that aren't choices (the mount rotation's derivation), and anything systemd
owns (`/etc/default/thermal-live` is per-deployment, not per-checkout).

### 4.3 Detection: `detector/`

Two implementations of the **same** white-top-hat + signal-to-clutter test,
sharing every threshold from config §2 on purpose (a drifted threshold would
make their outputs incomparable):

- **`tophat_scr.py`** — the original. A morphological opening with a 55×55
  **ellipse** structuring element reconstructs the smooth background (sky,
  horizon), subtracted off to leave a top-hat response; a candidate survives
  only if that response towers over the response in an annulus of background
  around it (signal-to-clutter). Costs ~703 ms on a full 1280×1024 frame on
  the Pi 5 — the opening is 91–95% of it, and OpenCV has no decomposition for
  an ellipse SE, so cost is exactly linear in `pixels × SE area`.
- **`boxtophat_scr.py`** — the same test made affordable: a **rectangular** SE
  (which OpenCV *does* run separably, two 1-D passes), integer box sums
  instead of float32 means, and a vectorised `np.maximum.at` instead of a
  per-component Python loop. Runs in ~27 ms on a full frame (37 fps mean).
  The square SE is uniformly *more* sensitive than the ellipse it's contained
  in (proven: `tophat_rect >= tophat_ellipse` at every pixel, verified
  `min(diff) = 0`), so nothing is lost — only which pixels clear the SCR gate
  shifts slightly (agrees with `tophat_scr` to within 4 px on 70% of
  detections, 25 px on 82%). **Not yet validated against ground truth** —
  only against `tophat_scr` on clutter frames with no real target in them.
  Re-run `thermal_detect.py --eval` before flying a change here.

Both expose `detect(frame) -> [(x, y, w, h, score), ...]`, auto-discovered by
every viewer/pipeline that scans the `detector/` folder.

### 4.4 Rejecting clutter: `filters/`

Run on the detector's raw output every frame, each exposing
`filter(boxes, context) -> boxes`:

- **`clutter_reject.py`** — two checks grounded in artifacts actually observed
  repeatedly in this footage, not theory: aspect ratio (real targets are
  blob-shaped; recurring false positives were 2×19 px / 53×2 px slivers), and
  frame-border contact (a bright region at the sensor edge, especially `x=0`,
  recurs across unrelated frames — consistent with a fixed sensor-edge defect).
- **`los_proximity.py`** — keeps only detections within
  `LOS_PROXIMITY_MAX_DIST_PX` of the attitude-only LOS prediction. Pass-through
  when no LOS reference exists yet (no click, no cue, no `.csv`).

### 4.5 Deciding what to start tracking: `initialisation/`

Each module exposes `init(boxes, context) -> (x, y) | None` — never moves,
weights, or substitutes for a real detection; it only *chooses between*
detections the detector already found. "No cue, or nothing near it" returns
`None`, which is the correct answer, not a failure.

- **`cue_nearest.py`** — **the flight default** (`config.FLIGHT_INITIALISER`).
  Acquires the detection nearest where the Cube's radar cue
  (`GCS_TARGET_BEARING`) projects to. Replaces `rightmost_isolated` for flight
  because, against ~13 false alarms/frame on real footage, that one locked
  onto clutter on frame 1 and — having no release mechanism — held it for
  840/841 frames of a bench test with no real target present.
- **`rightmost_isolated.py`** — **the viewer default**
  (`config.VIEWER_INITIALISER`), for offline review where there is no cue: the
  rightmost detection that is spatially isolated from every other candidate
  (sky is on the right of this rig's rolled mount; a distant target should
  show up alone out there).
- **`manual_click.py`** — acquires the detection nearest an operator's click on
  the live preview (`flight/preview.py`). Selected via `--initialiser
  manual_click` or `THERMAL_INITIALISER=manual_click`; the bench/no-radar
  fallback, or an override when the automatic rule picks the wrong blob out of
  a clutter field.

`config.CUELESS_INITIALISER` (default `rightmost_isolated`) is what runs when
the radar cue is entirely absent, so a dead radar link doesn't silently zero
out the whole detection uplink for the rest of the sortie — at the cost of a
possible clutter lock with no release authority, which the logged `acq_via`
field marks as a guess.

### 4.6 Tracking: `experiment/los_static_track.py`

The `SmoothTracker` class is the pipeline's core: it fuses an attitude-only
LOS prediction (reproject the last known reference point through the change
in gimbal/vehicle attitude since it was set, via a pinhole model) with
whatever detector output falls inside a rate-widened gate around that
prediction, blended with a hard per-frame pull cap. Config §5
(`TRACKER_BASE_GATE_PX`, `TRACKER_GATE_PX_PER_DEG_S`, `TRACKER_ALPHA_MAX`, …)
tunes the gate growth and blend. This is also where the file's original
purpose lives on: an **experiment** mode that draws a pure attitude-only
reprojection against real footage with no detector running at all, useful as
a sanity check on the mount/attitude math on its own.

The module is reused (not reimplemented) by `tools/rawrec_viewer.py`,
`tools/live_track.py`, `tools/telemetry_viewer.py` and
`tools/flight_pipeline.py` — all four import its SLERP/quaternion/reprojection
functions directly rather than keeping independent copies.

### 4.7 Single-object appearance tracking: `sot/`

- **`opencv_csrt.py`** — wraps OpenCV's CSRT correlation-filter tracker behind
  a `create()` factory exposing `init(frame, bbox)` / `update(frame)`. Given
  one bounding box on one frame, follows that patch of pixels forward by
  appearance alone — no detector, no attitude, independent of everything else
  in the repo. Wired into `tools/telemetry_viewer.py` (press `s`, drag a box)
  as a second opinion on a target the LOS/detector pipeline missed, or a check
  on where the LOS reprojection says the target actually went. Selected via
  `config.SOT_ALGO` so a different backend can be dropped in later without
  touching any caller.

### 4.8 The Cube link: `comms/`

The MAVLink connection to the CubeOrangePlus flight controller. The full wire
protocol is in **[RPI_COMMS.md](../RPI_COMMS.md)** — summary here:

| Direction | Message | id | Rate |
|---|---|---|---|
| Pi → Cube | `DETECTION_TARGET_DATA` | 42050 (custom) | one per detector frame, event-driven, ≥2 Hz to clear the firmware's 0.5 s staleness timeout |
| Cube → Pi | `ATTITUDE_QUATERNION` | 31 | 30 Hz, requested via `SET_MESSAGE_INTERVAL` (it is in no stream group, so no param can enable it) |
| Cube → Pi | `GCS_TARGET_BEARING` | 42051 (custom) | 5 Hz on request — the radar cue, already rotated into camera frame |

Modules:

- **`dialect.py`** — the *only* supported way to import `mavutil` in this
  repo. Two environment variables (`MAVLINK20=1`, `MAVLINK_DIALECT`) must be
  set **before** pymavlink is first imported anywhere in the process, or both
  custom message ids silently fail (send raises `AttributeError`; receive
  drops the message with no error at all). Raises if pymavlink was already
  imported, since by then it's too late to matter.
- **`gen_dialect.py`** — regenerates a *separate* pymavlink dialect
  (`thermal_link`, beside stock `ardupilotmega.py`, never overwriting it) from
  the XML in `comms/mavlink/`, so a `pip install --upgrade pymavlink` can't
  silently revert the link, and another MAVLink consumer on the same host
  keeps its own dialect untouched.
- **`comms/mavlink/*.msg.xml`** — the two custom message definitions,
  **reconstructed** from RPI_COMMS.md's field tables (the firmware repo's own
  `.msg.xml` isn't on this machine) and corroborated by matching CRC_EXTRA /
  documented wire size against the real Cube.
- **`bearing.py`** — pixel → camera-frame `(az, el)` for the uplink. The one
  rule that must not be gotten backwards: send the bearing in the **camera's**
  frame and let the Cube apply the mount **pitch** correction
  (`LAT_CAM_PITCH`) itself — but the firmware models pitch only, so mount
  **roll** must be removed *here* or nothing downstream ever removes it. This
  core is rolled 90° (`config.CAM_ROLL_DEG`), which exchanges the two image
  axes.
- **`los.py`** — the complementary case: a *world*-frame (NED) line of sight,
  with the mount rotation applied **in full** (roll and pitch both), because a
  world vector has no second party (the firmware) left to finish the job. The
  firmware currently decodes and **discards** the NED fields on the wire
  (`frame` must stay 0) — this exists for the flight CSV's own record and for
  whenever that firmware path is wired up.
- **`cube_link.py`** — `CubeLink`: one serial connection, a background reader
  thread, and an **attitude ring buffer** so `q_at(t)` can look *backwards* by
  the camera's own capture latency (~92–95 ms, from README.md) and SLERP
  between bracketing samples rather than reading "the latest quaternion" —
  nearest-neighbour was tried on the recorded path and went stale on ~30% of
  frames. Also holds `TargetCue` (the latest `GCS_TARGET_BEARING`) and
  `send_detection()`.

### 4.9 Flight scaffolding: `flight/`

Everything *around* the algorithm that a real sortie needs and a bench script
does not:

- **`camera.py`** — `FlightCamera`: stamps each frame in the grab thread
  (not downstream, where up to a frame period of unknown age would fold
  straight into the LOS latency compensation), serves *latest-only* to the
  processing path (so a slow detector drops frames instead of falling
  progressively further behind), but taps the recorder off the **grab**
  thread, not the processing path — so a raw capture is complete even while
  detection is too slow to look at every frame.
- **`rawrec.py`** — `RawRecWriter`: writes the `.rawrec` format (§7 below).
  Camera thread never blocks on disk (bounded queue, drop-and-count on
  overflow); refuses to open below a free-space reserve; no finalisation step,
  so a crash mid-write still leaves every prior frame readable (the old
  pipeline lost 5 of 27 recordings, including its two largest flights, to a
  format that needed a finalising index write).
- **`telemetry.py`** — `TelemetryWriter`: the same bounded-queue-plus-
  background-thread discipline as `RawRecWriter`, applied to the per-frame CSV.
  Added after a real sortie (2026-09-18, on the SD-card fallback) showed the
  main loop's own synchronous `csv_f.flush()` could block for up to 6.3 s at a
  time when the disk queue backed up — and because that call sat *after* the
  Cube uplink send in the loop, it didn't just delay the CSV, it froze
  detection, tracking and the uplink for every subsequent frame until it
  returned. `offer()` never blocks; a row is dropped and counted
  (`telem_dropped`, mirroring `raw_dropped`) instead.
- **`rc_arm.py`** — `RcArm` / `make_arm_switches()`: two independent RC
  switches (ch7 = algorithm, ch6 = record) with Schmitt-trigger hysteresis
  (a switch parked on the threshold must not flip every frame — each flip of
  the record switch opens/closes a `.rawrec` episode) and fail-safe disarm on
  any unknown state (no packet ever, packet timeout, or no link at all — all
  three are OFF, never "hold the last value").
- **`preview.py`** — `Preview`: the MJPEG page at `:8002`. A dedicated
  encoder thread so annotating/JPEG-encoding a 1280×1024 frame (which alone
  doesn't fit in the ~7 ms left of a 40 ms budget after detection) never
  costs the tracking loop a frame — `offer()` stores a reference and returns.
  Also serves the manual-click endpoint and runtime controls (arm, hot-swap
  detector/initialiser/filter) over `POST /api/control`.

### 4.10 Running it: `tools/`

- **`flight_pipeline.py`** — **the operational envelope.** Ties everything in
  §4.3–4.9 together, replicating the switch behaviour, episode naming, and
  fail-safe posture the rig already flew under, with this repo's own
  detection/tracking substituted in. `--no-uplink` runs everything and
  transmits nothing (the right setting for a first flight-line check);
  `--always-on` ignores the RC switches for bench work. Writes a session-meta
  file recording every parameter (and the exact git commit) a sortie actually
  ran with.
- **`live_track.py`** — the same pipeline against a **live** camera with
  **live** Cube attitude (no recording, no RC switches) — for bench-testing
  the full grab→attitude→LOS→crop→detect→filter→track→bearing chain before
  wiring it into the flight-mode envelope.
- **`rawrec_viewer.py` / `web_viewer.py` / `fb_viewer.py`** — three front ends
  (OpenCV window / headless HTTP / HDMI framebuffer) sharing one parsing,
  detector-loading, and LOS-reprojection core, for three different places a
  flight rig might be reviewed from (a laptop with a display, a laptop with
  only a network link, an HDMI monitor plugged into a headless Pi).
- **`telemetry_viewer.py`** — replays a `.rawrec` **beside** the `los-*.csv`
  the flight pipeline itself logged for it: every number drawn is what the
  pipeline actually decided at the time (no detector or filter is re-run),
  plus this file's own LOS reprojection drawn for comparison and an
  independent CSRT box for a second opinion. This is how a flight gets
  reviewed after the fact.
- **`cube_probe.py`** — walks RPI_COMMS.md §8's bench checklist against a
  real Cube in order, printing what it actually saw at each step (most
  failures here look identical from a distance — dead TX line, wrong baud,
  unregenerated dialect, firmware that doesn't implement a message — and this
  is what tells them apart). `--send-detection` is opt-in and refuses while
  the Cube reports ARMED.
- **`health_check.py`** — five independent checks
  (systemd state / journal freshness / device nodes / thermal throttling /
  disk space) for whether `thermal-live.service` is *actually doing something
  useful*, not merely "active". Exit code is the worst severity seen, so it
  drops into a cron job or a monitoring check unmodified.
- **`drop_report.py`** — was a frame dropped while recording, and confirmed by
  which layer? Cross-checks two independent signals for one or more `.rawrec`
  files: a timestamp-gap scan over the file's own frame index (ground truth
  for how many frames are actually in the file) against the matching
  `los-*.csv`'s `raw_dropped` counter (what the recorder itself believed at
  the time, per episode — `flight/rawrec.py`'s `RawRecWriter.dropped`). A csv
  row *inside* a flagged gap window means detection/tracking/uplink kept
  running through it; zero rows means the whole loop stalled, not only
  recording (`--list-gaps` for the per-gap detail either way).
- **`disk_soak.py`**, **`det_bench.py`**, **`tophat_profile.py`** — the
  benchmarking trio: can the recording disk sustain 25 fps with zero drops
  (driving the *real* writer, not `dd`, which only measures page-cache
  speed); per-stage detector cost on a real `.rawrec` (I/O separated from
  compute, thermal-throttle-checked); and `tophat_scr` broken open
  stage-by-stage with priced alternatives, each cross-checked for behaviour
  parity against the real detector, not just speed.
- **`rawrec2mp4.py`** — converts a `.rawrec` to H.264 for human review only
  (never delete the original — every reader in this repo needs the raw
  pixels, and H.264 has already destroyed the values the detector
  thresholds on). Places frames on a *real* timebase from measured `t_mono`
  deltas, not the header's `advertised_fps` (which lies) and not a frame
  counter (which would silently speed through a real recording gap).

### 4.11 Deployment: `deploy/`

Fully documented in **[deploy/README.md](../deploy/README.md)**; the Cube-link
bring-up half is in **[deploy/COMMS.md](../deploy/COMMS.md)**. Summary:

```bash
bash deploy/install.sh --check      # preflight only, writes nothing
bash deploy/install.sh --start      # install + start
bash deploy/setup_comms.sh          # once, before using track/flight mode
sudo systemctl start thermal-live
journalctl -u thermal-live -f
```

Four modes, selected by `THERMAL_MODE` in `/etc/default/thermal-live`
(reference copy: `deploy/thermal-live.env`):

| Mode | What runs | Attitude source | Rate (measured) |
|---|---|---|---|
| `web` | detector + clutter rejection only, HTTP viewer | none — no LOS, no tracker, no ROI crop | ~1.3 fps (full-frame search every frame) |
| `track` | the full pipeline, headless | live, 30 Hz from the Cube | 6.6–11.1 fps once locked (the tracker's gate makes a ROI crop possible) |
| `flight` | `track` + the two RC switches + `.rawrec` episodes + per-frame CSV | live | same as `track` |
| `fb` | raw feed (or one detector) on the attached HDMI display | none | up to 30 fps, no detection cost gates it |

`thermal-live.service` never uses `BindsTo=` on the camera device
deliberately (that would tie it to a udev rule it doesn't own); instead
`deploy/run_live.sh` polls for the device, refuses to start a second reader
while another process holds the camera, and — the failure mode that matters
most — watchdogs the device node so a camera that silently vanishes from the
USB bus (which otherwise leaves the grab thread parked forever in a 10 ms
retry loop, `Restart=always` never firing) gets killed and restarted rather
than serving a frozen frame under a service that still reports "active".

---

## 5. `thermal-live-deploy` — the stripped deployment checkout

One commit, `06c7843`, on top of `flight-pipeline`'s `d816a3f`. It keeps
exactly the exec/import closure reachable from `deploy/run_live.sh` across all
four modes, plus the same-project tooling for operating that deployment
(`health_check.py`, `cube_probe.py`, `tophat_profile.py`, `disk_soak.py`,
`rawrec2mp4.py`, `telemetry_viewer.py`, `sot/`) — see §1's table for the full
removed-file list and the commit message for the per-file rationale (in
particular why `tools/det_bench.py` couldn't be kept: it unconditionally
imports `thermal_detect` and `rawrec_view.Capture` as its primary subject, not
just for its optional `--tophat` comparison).

**Nothing here is independent development.** The commit message is explicit:
this is not a judgement that the Jetson-era latency-rig tooling matters less —
both trees still exist, in full, on `flight-pipeline`. This branch exists
purely so a deployment checkout is *provably* minimal: every file on it is
something `thermal-live.service` can actually reach, and nothing else.

Because it was cut before `flight-pipeline`'s three most recent commits, it is
**missing**: the SOT-tracker/cached-indexing/telemetry-export additions to the
viewers (`717f759`), the PMIC voltage/rail-draw readout in the web viewer
(`4957050`), and the `cube_text` `NameError` crash fix in
`tools/telemetry_viewer.py` (`28f9eec`). If this branch is redeployed, check
first whether those should be cherry-picked or whether the branch should be
recut from current `flight-pipeline`.

---

## 6. Cross-branch file map

Files that exist **only on `main`** (the Jetson latency rig; absent from both
RPi5 branches):

```
bin/  calibration/  docs/THERMAL_SERIAL_FAULT.md  docs/important_comands.txt
dets.csv  rawrec_3d.py  rawrec_annotate.py  rawrec_view.py  read_core.py
test_mac.txt  thermal_detect.py  tools/blob_log.c  tools/imu_live.c
tools/record_raw.c  tools/set_fps_jetson.py  tools/view_camera.c
```

Files added on **`flight-pipeline`** (present there and, mostly, on
`thermal-live-deploy` too — see below): `RPI_COMMS.md`, `comms/`, `deploy/`,
`detector/boxtophat_scr.py`, `experiment/`, `filters/`, `flight/`,
`initialisation/`, `sot/`, and the new `tools/` modules (`cube_probe.py`,
`det_bench.py`, `disk_soak.py`, `fb_viewer.py`, `flight_pipeline.py`,
`health_check.py`, `live_track.py`, `telemetry_viewer.py`,
`tophat_profile.py`).

Files present on **`flight-pipeline` but *not* `thermal-live-deploy`**: every
`main`-only file above, **plus** `README.md`, `.DS_Store`, `config.py`'s
7 flight-pipeline-only lines trimmed, and `tools/det_bench.py`.

`config.py`, `detector/tophat_scr.py`, `experiment/los_static_track.py`,
`docs/DEVICE_NODES.md`, `tools/rawrec_viewer.py`, `tools/web_viewer.py`, and
everything under `comms/`, `deploy/`, `flight/`, `filters/`,
`initialisation/`, `sot/` are shared, near-identical content across
`flight-pipeline` and `thermal-live-deploy` (`thermal-live-deploy`'s copies
are simply frozen at `d816a3f`).

---

## 7. Data formats reference

### On-disk layout: `logs/`

Flight-pipeline output is organised by file type, not flattened into one
directory:

```
logs/
  rawrec/     flight-<stamp>-NN.rawrec        raw captures (RawRecWriter)
              flight-<stamp>-NN.rawrec.idx    per-recording frame-offset cache
                                              (tools/rawrec_viewer.py's build_index();
                                               disposable, regenerated automatically)
  telemetry/  los-<stamp>.csv                 per-frame flight telemetry
              los-<stamp>.meta.json           the sortie's full parameter set
  video/      flight-<stamp>-NN.mp4           .rawrec -> mp4 review copies
              <...>_annotated.mp4             (tools/rawrec2mp4.py, never the source of truth)
```

- **`rawrec/` + `telemetry/` are created on every start** by
  `deploy/run_live.sh`'s `resolve_log_dir()`, on whatever filesystem
  `THERMAL_LOG_DIR` (or its fallback) resolved to — the existing free-space
  reserve check already covers both, since it's a check on that parent
  directory, not on a specific subfolder.
- **`los-<stamp>.meta.json` always lands beside its `.csv`**, wherever that is
  — `tools/flight_pipeline.py`'s `write_session_meta()` derives its path from
  `--out-csv` directly (`os.path.splitext(out_csv)[0] + ".meta.json"`), so
  the two never need separate configuration.
- **`.rawrec.idx` always lands beside its `.rawrec`** for the same reason —
  `_index_cache_path()` is `str(rawrec_path) + ".idx"`, so it follows the
  `.rawrec` into `rawrec/` automatically.
- **Cross-referencing a `.rawrec` to its `.csv`** (`experiment/los_static_track.py`'s
  `find_los_csv()`, used by every viewer and by `rawrec2mp4.py --telemetry`)
  checks, in order: the same directory as the `.rawrec`; the sibling
  `telemetry/` next to a `rawrec/` one; then a recursive search under the
  `.rawrec`'s parent and grandparent. This still finds a CSV in an
  arbitrarily-named subfolder (a `no_drone/`/`drone/` split, say) — it does
  not require the `rawrec/`+`telemetry/` convention, it just resolves it in
  the fewest filesystem calls when that convention *is* what's on disk.
- **`tools/rawrec2mp4.py`** without `-o`/`--out` writes into the sibling
  `video/` directory automatically when its input's parent directory is named
  `rawrec` (creating `video/` if needed); otherwise it falls back to writing
  beside the input, unchanged from before this layout existed.

### `.rawrec` — the raw capture format (`flight/rawrec.py`)

Self-describing, so no sidecar file is needed (unlike `record_raw.c`'s
`.gray`+`.meta`+`.idx` triple on `main` — **do not confuse the two formats**,
despite both holding raw sensor bytes):

- **Bytes 0–4095**: file header. `8s` magic `b"RAWREC\x00\x01"` at `0x000`;
  `IIIIII` (`hdr_len, rec_len, width, height, bpp, frame_bytes`) at `0x008`;
  `8s` pixfmt at `0x020`; `dd` (`t_wall, t_mono` at open) at `0x028`; a
  NUL-terminated JSON descriptor at `0x080` (width/height/bpp/pixfmt/
  frame_bytes/rec_len/advertised_fps/focal/episode/out_csv).
- **Then N records** of `rec_len = 32 + frame_bytes` bytes each: `I` magic
  `0xA5F00DEC`, `Q` seq, `d` `t_mono` @0x0C, `d` `t_wall` @0x14, `H` flags
  @0x1C, `H` crc16 @0x1E, then the frame payload.
- **No finalisation step, deliberately** — each frame is fully readable the
  instant it's written; there is no index or trailer to lose in a crash.
- **`advertised_fps` in the header lies** (a real recording can carry
  `advertised_fps=50.0` while every reader measures 25 fps from `t_mono`
  deltas) — every consumer derives the true rate from the timestamps, never
  the header.

### `blobs.csv` (main-branch `blob_log.c`) — 28 named columns

```
seq, t_first_byte, t_available, latency_ms,
yaw, pitch, t_imu, imu_dt_ms, n_imu_interval,
valid, cx, cy, area_px, blob_peak, frame_peak, thr, ncomp, second_area,
x0, y0, x1, y1, queue_ms, det_ms, t_detect_done, fb_to_det_ms,
flags, det_state
```

Looked up **by name**, never by position, everywhere it's read — the file has
gained columns twice already. All `t_` columns are `CLOCK_MONOTONIC`.

### `los-*.csv` (flight-pipeline `tools/flight_pipeline.py`)

Per-frame flight telemetry: predicted LOS point, tracking/coasting state,
gate radius, detector box counts before/after filtering, the ROI crop
searched, the attitude used, the bearing sent, and the Cube's own cue —
everything the pipeline decided, at the time, joined to the matching
`.rawrec` by `t_mono`/`rec_idx`. This is what `tools/telemetry_viewer.py`
replays and what `tools/rawrec2mp4.py` burns into an mp4 overlay.

### `imu.csv` (main-branch `blob_log.c`)

Every telemetry frame that passed CRC: `t_imu` plus 8 floats
(`yaw, pitch, ...`), same `CLOCK_MONOTONIC` as `blobs.csv`'s frame times.

---

## 8. Quick command reference

```bash
# --- Jetson latency rig (main; also builds fine from flight-pipeline) ------
./bin/blob_log -t 30 -S 2                          # record while panning
./bin/det_latency -d recordings/<date>/<time> -v   # measure the latency
./bin/imu_live                                     # live px/frame coaching (NOT with blob_log running)

# --- RPi5 flight pipeline (flight-pipeline / thermal-live-deploy) ----------
python3 tools/cube_probe.py                        # bench-checklist the Cube link
bash deploy/setup_comms.sh                          # bring up the Cube UART + dialect
bash deploy/install.sh --check                      # preflight the systemd unit
python3 tools/live_track.py --no-uplink             # bench the whole live chain, transmit nothing
python3 tools/flight_pipeline.py --no-uplink \
    --out-csv logs/telemetry/los-S.csv --raw-video logs/rawrec/flight-S.rawrec
python3 tools/telemetry_viewer.py logs/rawrec/flight-S-01.rawrec  # auto-finds logs/telemetry/los-S.csv
python3 tools/health_check.py --json                # is the live service actually healthy?
python3 tools/drop_report.py logs/rawrec/*.rawrec   # any frames dropped while recording?
python3 tools/rawrec2mp4.py logs/rawrec/flight-S-01.rawrec # -> logs/video/, auto. Review only — never delete the .rawrec
```
