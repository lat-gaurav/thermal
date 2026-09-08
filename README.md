# thermal

Data acquisition and latency measurement for a thermal camera rigidly mounted on
a gimbal, running on a Jetson Orin Nano.

The rig records three streams against **one clock**: thermal video, the gimbal's
IMU at ~1 kHz, and a blob detection per frame. Everything is C against raw V4L2
and termios — no OpenCV, no libv4l2 (it silently converts pixel formats), and no
Python in any path that touches a frame.

The headline result the tooling exists to produce:

| segment | measured |
|---|---|
| **[A]** world moves → frame's first byte reaches the host | **92–95 ms** |
| **[B]** first byte → frame complete and available | 35.4 ms |
| **[C]** available → detector starts | 0.03 ms |
| **[D]** blob detection | 18.2 ms (`-S 1`) / 4.9 ms (`-S 2`) |
| **total** world movement → `cx,cy` exist | **~147 ms** (~134 ms with `-S 2`) |

`[A]` is the camera's own hidden latency and no host clock can see it. It is
measured by using the gimbal IMU as an external time reference — see
[Measuring latency](#measuring-latency).

---

## Hardware

| | |
|---|---|
| Host | Jetson Orin Nano, p3768 carrier, L4T R36.4.7 |
| Camera | Artosyn "Sirius" thermal core, `1d6b:0101` |
| Stream | 1280×1024 GREY, 25 fps, 1.31 MB/frame |
| Sensor | **640×512** — the 1280×1024 stream is a 2× upscale |
| Lens | **27.3 px/deg** (46.8° HFOV), measured, not assumed |
| Bus | USB 2.0 high-speed, 480 Mbps |
| Gimbal | telemetry over an FTDI FT232R at 921600, ~1010 Hz |

Two facts worth internalising before using any of this:

**The camera is bandwidth-limited, not frame-rate-limited.** 1280×1024 at 25 fps
is 32.8 MB/s. At 50 fps it would need 65.4 MB/s, above even USB 2.0's 60 MB/s
theoretical ceiling — which is why the measured maximum is ~33 fps (43.6 MB/s),
right at the practical limit.

**Three quarters of every frame is interpolated.** The sensor is 640×512. `-S 2`
in the detector discards those interpolated pixels and costs nothing real, while
cutting detection from 18.2 ms to 4.9 ms.

---

## Build

No build system; each tool is one translation unit.

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

All eight compile with zero warnings. `libjpeg-dev` is the only non-libc
dependency. The Python tools need `pyserial`.

---

## Tools

### Recording

**`blob_log`** — the main recorder. Logs IMU at ~1 kHz and a blob detection per
frame, with an annotated live preview, under a strict priority order:

```
frames  >  IMU logging  >  blob detection  >  preview
```

Nothing lower can block anything higher, structurally rather than by tuning. The
acquisition thread only ever does `DQBUF → snapshot → pair an IMU sample →
QBUF`; every branch out of it ends in a `QBUF`, so no downstream stage can hold
a buffer hostage. Only the preview is designed to lose data.

```bash
./bin/blob_log -t 30 -S 2            # 30 s, cheap detection, preview on :8080
./bin/blob_log -S 2 -p 0             # no preview, lowest load
```

Writes `recordings/<date>/<time>/` → `imu.csv`, `blobs.csv`, `summary.txt`.

Measured over a 15 s run with a browser attached: **0 frames dropped**, 25.01 fps,
IMU 1009.8 Hz with 0 CRC errors, 100% of frames detected on, preview 25.01 fps.
Frame↔IMU pairing: mean |dt| **0.29 ms**.

**`record_raw`** — every bit of raw sensor data to disk, no compression, no
detection. Use when you want the pixels themselves.

### Live monitoring

**`imu_live`** — live yaw/pitch plus angular rate, with a verdict on whether the
pan speed is one the tracker can follow. It shows **px/frame**, which is the
quantity that actually decides whether a recording is usable.

```bash
./bin/imu_live          # pan until it reads IDEAL
```

**`view_camera`** — MJPEG preview over HTTP with no detection.

> **Never run `imu_live` while `blob_log` is recording.** Both read the same tty
> and each steals the other's bytes. Measured: `blob_log` drops to 780 Hz with
> 182 CRC errors. `imu_live` detects this and warns.

### Analysis

**`det_latency`** — the latency measurement. See below.

**`detect_blob`** — offline blob detection and oscillation analysis over a
finished `.gray`, with optional annotated video output.

**`flow_stamp`** — per-frame optical flow by 1-D profile alignment. Superseded by
`blob_log` for latency work; kept because its dual-timestamp and flow columns are
still the reference for frame timing.

**`jerk_latency`** — onset-pairing latency estimator. **Superseded and not
recommended** — see [What did not work](#what-did-not-work).

### Camera control

| tool | purpose |
|---|---|
| `set_fps_jetson.py` | set/query frame rate; resolves nodes by USB serial |
| `read_core.py` | read every documented read-only register; `--selftest` |
| `check_gimbal.py` | six-stage gimbal link diagnosis |
| `set_fps.py` | **original RPi script — do not edit.** Hardcodes `/dev/thermal0`, which does not exist here |

---

## Measuring latency

A host clock cannot see when the world moved — only when bytes arrived.
Everything between the photons and the first USB byte is invisible from this
side, and on this camera it is the largest term in the chain.

The gimbal IMU solves it. It reports its own angle at ~1 kHz on the same
`CLOCK_MONOTONIC` as the frames, and that report is effectively instantaneous
next to the video path. The camera is bolted to the gimbal, so for a world-fixed
scene the image *must* move as a fixed linear function of gimbal angle. Sweep a
time shift τ backwards, fit that relation at each τ, and the τ that minimises the
residual is the latency.

```bash
./bin/blob_log -t 30 -S 2                              # record, while panning
./bin/det_latency -d recordings/<date>/<time> -v       # measure
```

### Why a joint 2×2 fit

The mount is rigid, so image x and y are not two independent observations — they
are one rotation. `det_latency` fits the whole map at every candidate τ:

```
[ vx ]   [ b00  b01 ] [ yaw_rate   ]     loss(τ) = SSres/SStot = 1 − R²
[ vy ] = [ b10  b11 ] [ pitch_rate ]     minimised over τ
```

This uses all the motion instead of half, tolerates any roll misalignment, and —
the real reason — buys a **free physical check**. A rigid mount can only produce
`scale × rotation`, which forces `b00 = b11` and `b01 = −b10`, and `sqrt|det B|`
must equal the lens's px/deg. Nothing in the fit imposes either. A deep minimum
with a non-conformal matrix or an absurd scale is coincidence, and the tool says
so rather than printing a number.

It fits **rates, not positions**: both signals carry slow drift that would
otherwise dominate and drag the minimum toward τ = 0.

### It refuses rather than guesses

`det_latency` exits non-zero and explains itself when the gimbal did not move,
the correlation is too weak, the minimum sits at the edge of the sweep, or the
fitted matrix is not a rigid rotation. It also reports a **spread across
independent blocks** instead of a bare number.

**Validated against synthetic data with known lag:** 0 / 20 / 45 / 80 / 120 ms
recovered to within **1.6 ms**. Injecting 18 rollovers through ±180° changed
nothing (60.0 ms → 60.0 ms), confirming the wrap handling.

### Recording for it — the two rules that matter

**1. Keep the total sweep under ~33°.** The FOV is 46.8°. A 51.8° sweep demands
1400 px of travel through a 1280 px frame, so the target *must* leave the frame,
after which the detector locks onto a different hot object — 747 px `cx` jumps
that no gimbal motion can explain. This wrecked one run (R² 0.645); a 45° sweep
gave R² 0.998.

**2. Peak rate 30–40 deg/s.** Above ~137 deg/s the tracker loses lock outright.

`det_latency` rejects track breaks automatically at 4σ and re-sweeps, which alone
took that bad run from R² 0.645 to 0.9986.

---

## What did not work

Kept deliberately, because the failures were expensive and are easy to repeat.

**Jerk-onset latency estimation** gave 1.91, 3.05, 5.86, 5.84, 2.72 frames across
runs — a between-run spread far larger than the within-run scatter. The root
cause was not the estimator's thresholds. At 270 deg/s the image shifts ~200 px
between frames and smears, so the tracker matches the wrong feature *while
reporting high quality*:

```
dx_px =   0.01   while d_yaw = 9.668 deg    (expected ~180 px)
dx_px = 254.04   while d_yaw = 0.725 deg    (expected ~13 px)
```

Filtering to the slow frames cannot rescue such a record — the only frames below
the limit are the turnaround instants. **A violent jerk is the worst case for
this measurement, not the best.**

**Two processes on one serial port.** Every byte one reads is a byte the other
never sees. Measured cost: 1009 Hz → 780 Hz with 182 CRC errors.

**Assuming the lens scale.** An assumed 18.6 px/deg was wrong; the fit measured
27.3 px/deg across two independent runs. `det_latency` now treats a scale
mismatch as a warning, not a veto — a wrong assumption should not discard a good
measurement.

---

## Device nodes

`videoN`, `ttyACMn` and `ttyUSBn` are assigned in enumeration order and **do
move** — during one debugging session the camera cycled through device numbers
4 → 6 → 9 → 11 → 4 → 24 and changed hub port, after which the FTDI took the slot
the camera had occupied.

Every tool resolves by USB serial through a fallback chain and never by kernel
name. See **[DEVICE_NODES.md](docs/DEVICE_NODES.md)** for the full topology, the
`video0`/`video1` metadata-node trap, and the udev rules.

**[THERMAL_SERIAL_FAULT.md](docs/THERMAL_SERIAL_FAULT.md)** documents a four-day
outage of the camera's serial channel caused by ModemManager AT-probing the
CDC-ACM port, and the udev rule that fixes it
(`99-thermal-core-mm-ignore.rules`, installed to `/etc/udev/rules.d/`).

**[important_comands.txt](docs/important_comands.txt)** is the working command
reference — recipes, network setup, and the recording rules above.

---

## Output formats

`blobs.csv` — one row per acquired frame, 28 columns. Columns are looked up **by
name** by every consumer, because this file has gained columns twice.

```
seq, t_first_byte, t_available, latency_ms,
yaw, pitch, t_imu, imu_dt_ms, n_imu_interval,
valid, cx, cy, area_px, blob_peak, frame_peak, thr, ncomp, second_area,
x0, y0, x1, y1, queue_ms, det_ms, t_detect_done, fb_to_det_ms,
flags, det_state
```

`imu.csv` — every telemetry frame that passed CRC: `t_imu` plus 8 floats.

All `t_` columns are `CLOCK_MONOTONIC`, verified against the V4L2 buffer flags
rather than assumed, so IMU and frame times are directly comparable.

- `t_first_byte` — kernel stamp, first USB payload of the frame on the host
- `t_available` — `VIDIOC_DQBUF` returned it
- `t_detect_done` — `cx,cy` existed; the detection **output** time

`cx,cy` are positions **in the image**: blob motion plus camera motion. The
`yaw`/`pitch` columns beside them are what let you separate the two.

---

## Open questions

**What is the 92 ms made of?** Candidates: the bolometer's thermal time constant
(~10 ms, fixed), integration and readout, pipeline stages, a temporal noise
filter, and the core's output buffer waiting for its USB slot. The decisive
experiment is `[A] = fixed + k · T_period` — measure at a second frame rate and
solve. 50 fps is impossible on this bus, but 25 → ~33 fps is enough to separate
the fixed from the rate-scaled part.

**Is there a rolling-shutter gradient?** Observed informally as ~100 ms at the top
of the frame and ~80 ms at the bottom, implying a ~20 ms top-to-bottom scan. The
sign is what a progressive readout predicts, and the midpoint agrees with the
92–95 ms measured at frame centre. **Not yet confirmed** — both existing
recordings have the blob at essentially one height (`cy` sd of 7 and 13 px out of
1024), so there is no vertical leverage. `det_latency -Z N` measures it once a
recording exists where the blob traverses the frame vertically.
