# deploy/ — running this repo's pipeline on the live camera

Everything here is self-contained: the unit points back into this checkout, and
nothing in it depends on any other project on the host.

```bash
bash deploy/install.sh --check      # preflight only, writes nothing
bash deploy/install.sh              # install, do not start
bash deploy/install.sh --start      # ... and start now
bash deploy/install.sh --enable     # ... and at every boot too
bash deploy/install.sh --uninstall  # remove it again
```

| file | installed to | what it is |
|---|---|---|
| `thermal-live.service` | `/etc/systemd/system/` | the unit; `@REPO@`/`@USER@` are filled in at install time |
| `thermal-live.env` | `/etc/default/thermal-live` | every tunable, one file. **Existing copies are never overwritten** |
| `run_live.sh` | *(stays here)* | what the unit execs: waits for the devices, refuses to share them, sets the frame rate, supervises |
| `setup_comms.sh` | *(stays here)* | brings up the Cube link: UART, groups, pymavlink dialect, then the live checklist |
| `COMMS.md` | *(stays here)* | the Cube link: wiring, the four silent failures, what was measured |
| `../99-thermal-core-mm-ignore.rules` | `/etc/udev/rules.d/` | keeps ModemManager off the core's serial port (`docs/THERMAL_SERIAL_FAULT.md`). Not currently in `/etc` on this host, and ModemManager is not installed here either — so it is a no-op guard today, shipped so the four-day outage cannot come back with the package |

Backups of whatever was in `/etc` first land in `deploy/backups/<stamp>/`.

`install.sh` does **not** start the service. Starting it takes the camera, and
the core permits exactly one streaming reader — that is a decision for whoever
is at the rig, not a side effect of copying a file.

---

## What actually runs live

There are two live modes, and the difference between them is an attitude source.

**`THERMAL_MODE=track` runs the whole pipeline.** `comms/` supplies live camera
attitude from the Cube at 30 Hz on the same `CLOCK_MONOTONIC` the frames are
stamped with, so `SmoothTracker` and `filters/los_proximity` work on a live feed
exactly as they do over a recording — and the tracker's gate makes the ROI crop
possible, which is what makes the rate usable. See [COMMS.md](COMMS.md) for
bringing that link up.

**`THERMAL_MODE=web` is detector + clutter rejection only.** A viewer has no
attitude source: `web_viewer.py --live` reports `has_los: false`, builds no
tracker, and `los_proximity` sees `los_point = None` and passes everything
through. `initialisation/` cannot run either, since it seeds a tracker that does
not exist.

| stage | `track` | `web` | over a recording |
|---|---|---|---|
| `detector/tophat_scr` | yes | yes | yes |
| `filters/clutter_reject` | yes | yes | yes |
| `filters/los_proximity` | yes | pass-through | yes |
| `SmoothTracker` + ROI crop | yes | **no** | yes |
| `initialisation/rightmost_isolated` | yes | **no** | yes |
| bearing uplink to the Cube | yes | no | no |

## Cost: the ROI crop is what makes this affordable

Measured on this Pi 5, OpenCV 4.10.0 with 4 threads, 2026-09-09:

| | |
|---|---|
| core delivers | **25.7 fps** GREY 1280×1024 (77 frames in 3.0 s) |
| budget at 25 fps | **40 ms/frame** |
| `tophat_scr.detect`, full 1280×1024 | **703 ms** mean (10 reps) |
| same, end to end through the web viewer | **0.72–0.90 s** per frame |
| same, on a tracker-sized ROI crop | **83–146 ms** (520×300 to 520×520) |
| detector off (`none`) | **0.02 s** per frame |

So, measured end to end over 20–25 s runs:

| mode | achieved |
|---|---|
| `web`, detection on | **~1.3 fps** — full frame every frame |
| `track`, once locked | **6.6–11.1 fps** — 132/134 and 278/279 frames tracking |
| `track`, while acquiring | ~1.4 fps — the search is necessarily full-frame |

Full-frame detection is ~18× over budget, which is far worse than the
"~0.2 s/frame" figure in `tools/fb_viewer.py`'s docstring; that number does not
hold at 1280×1024 on this machine. The crop is not an optimisation bolted on
afterwards — the tracker's gate is what defines it, so LOS and affordability
arrive together or not at all. The spread in achieved rate is just the gate's:
it grows with angular rate and with frames since the last fix.

Why `web` mode tolerates being slow, and `fb` would not: `LiveSource` grabs in
its own thread and serves *whatever is latest*, so a slow detector drops frames
instead of queueing them and lag stays bounded at one frame. `fb` mode redraws
on a timer, which is why `THERMAL_FB_DETECT` is empty by default.

`tools/det_bench.py` re-measures this properly (percentiles, per-stage
breakdown, and it checks whether the core throttled mid-run) as soon as there is
a `.rawrec` on the box to run it against.

## `flight` mode — the operational envelope

`THERMAL_MODE=flight` runs [tools/flight_pipeline.py](../tools/flight_pipeline.py),
which reproduces how this rig already flew, so a sortie behaves the way the crew
expects. What carries over, and what deliberately does not:

| | old service | `flight` mode |
|---|---|---|
| algo switch | RC ch7 > 1750 µs | **same** |
| record switch | RC ch6 > 1494 µs | **same** |
| switch hysteresis | ±50 µs Schmitt | **same** |
| RC-loss failsafe | 2 s → OFF | **same** |
| the two switches | fully independent | **same** — all four combinations |
| recording | `.rawrec`, one episode per switch-on | **same**, `-NN` suffix |
| disk guard | `--raw-reserve-mb 2048` | **same** |
| recorder fed from | the camera thread | **same** |
| file naming | `date-bootid`, shared by CSV and raw | **same** |
| per-frame CSV | `los-<stamp>.csv` | **same name and clock**, new columns |
| tracker on re-arm | rebuilt clean | **same** |
| priority | `Nice=-10`, best-effort IO | **same** ([10-flight.conf](10-flight.conf)) |
| retry policy | forever, no start limit | **same** |
| detection | its own | **this repo's** `detector/` + `filters/` |
| tracking | Kalman + peak/centroid latch | **this repo's** `SmoothTracker` |
| stabilisation | frame warp / ego-motion | **absent** — see below |
| horizon crop / draw | yes | **absent** |
| MJPEG preview | port 8000 | **absent** — use `web` mode |

**On-disk layout.** File *names* are unchanged from the old service, but
`run_live.sh` now writes them into type subfolders under `$THERMAL_LOG_DIR`:
raw captures to `rawrec/flight-<stamp>-NN.rawrec`, the per-frame CSV and its
`.meta.json` sidecar to `telemetry/los-<stamp>.csv` — created on every start,
on the same filesystem `$THERMAL_LOG_DIR` already resolved to, so the existing
disk-reserve check covers both. `tools/rawrec2mp4.py` follows the same
convention: run against a file under `rawrec/`, its `.mp4` lands in the
sibling `video/` directory automatically. See
[docs/REPOSITORY_GUIDE.md](../docs/REPOSITORY_GUIDE.md) for the full layout.

### What is genuinely missing, not just renamed

The old pipeline compensated for camera rotation by warping the frame or by
handing an inter-frame rotation to its tracker. This one instead reprojects its
LOS reference through the attitude quaternion each frame, which addresses the
same problem by a different route — it is not a port of that mechanism. There is
also no `--latch`, no peak/centroid mode switching, no horizon crop, and no
`--det-*` gate: the detection gate here is `config.DETECTOR_MIN_SCR` plus
`filters/`. **Do not assume the old detection tuning transfers**; none of those
numbers have an equivalent here.

### Verified against a simulated Cube, 2026-09-10

The Cube was powered off, so the whole envelope was driven over a UDP loopback
with scripted RC values:

- ch6 on alone → `REC-ONLY`, episode 01 opened, **579 frames / 0.76 GB, 0 dropped**
  written while the algorithm was off
- ch7 on → `RUNNING`, 25.0 fps, 7 ms/frame, tracking, uplink counting
- ch7 off → algorithm stops, **recording continues**
- ch6 off → episode saved; ch6 on again → **new episode 02** (937 frames)
- both episodes reopen in this repo's own readers at **25.00 fps**
- the CSV is `los-*.csv` compatible: `load_quaternions()` read **1170** attitude rows
- record switch on with no `--raw-video` → episode stays 0, no log spam
- reserve breached → refuses with the free/required figures, **keeps flying**, writes nothing

Two bugs that run found and that are now fixed: the episode counter incremented
once per frame whenever a recorder failed to open, and the status line divided
processed frames by total runtime, reading ~6× low after any spell of idling.

## The three things `run_live.sh` handles

1. **The device node arrives late.** USB enumeration is slower than boot, so it
   polls for `$THERMAL_DEVICE` for `THERMAL_WAIT_SECS`.
2. **One reader only.** It waits for the camera to be free and, if it is still
   held, exits naming the holder rather than opening a second reader that
   silently gets no frames. `Restart=always` + `RestartSec=5` means it keeps
   trying, so it takes over on its own the moment the other reader lets go.
3. **The camera can vanish without the viewer noticing.** When the core drops off
   the bus, the grab thread gets `ok=False` and sleeps 10 ms *forever* — it never
   exits, so `Restart=always` never fires and the service stays "active" serving
   a frozen JPEG. The watchdog loop notices the node is gone, kills the viewer
   and exits non-zero, which turns a silent hang into a restart.

`BindsTo=dev-thermal0.device` would cover (3) more cheaply, but it ties this unit
to a udev rule this repo does not own, so the polling loop is deliberate.

## Verified on hardware, 2026-09-09

- `install.sh --check` — all preflight items pass on this box
- `run_live.sh` in `web` mode — serves the live core, `is_live: true`, 1280×1024
- the frame-rate step — real serial exchange with the core, `status 0x00`, code echoed
- `SIGTERM` (what `systemctl stop` sends) — viewer stops, camera released
- device-vanish watchdog — node removed under it, exit 1, camera released
- exclusivity — a second instance refuses and names the holding PID

Not verified: `fb` mode. **`/dev/fb0` does not exist on this host** as it
currently boots, so `THERMAL_MODE=fb` refuses to start until a display is
attached and the console comes up on KMS.

## Day to day

```bash
bash deploy/setup_comms.sh             # once, before using track mode
sudo systemctl start thermal-live      # take the camera (and the UART, in track mode)
sudo systemctl stop thermal-live       # hand them back
journalctl -u thermal-live -f
```

In `track` mode there is no web page; the journal carries the status line, and
`THERMAL_TRACK_CSV` logs one row per frame. Set `THERMAL_UPLINK=0` for a first
flight-line check: the whole pipeline runs and nothing reaches guidance.

Then open `http://<pi>:8001/` and use the page's buttons: **cycle detector**
(`tophat_scr` → `none`) and **toggle annotate**. Port is 8001, not 8000, because
another service on this host already binds 8000.

## A note on libcamera

If "libcamera" was the intent rather than the live V4L2 feed: it is installed
here (`libcamera 0.7.1`, `rpicam-apps 1.12.0`) but `rpicam-hello --list-cameras`
reports **"No cameras available!"**, and neither `picamera2` nor the `libcamera`
Python module is installed. The thermal core is a USB UVC device, which is not
what the Raspberry Pi camera stack is for. Every frame path in this repo —
`web_viewer.py`, `fb_viewer.py`, `record_raw.c`, `blob_log.c` — is raw V4L2, so
V4L2 is what this deployment uses.
