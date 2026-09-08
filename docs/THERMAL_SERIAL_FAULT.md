# Thermal Core Serial Fault — root cause and fix

**Status: RESOLVED** · SD-IRM-256-01 on Jetson1 · 27–31 Aug 2026

For four days the thermal core's serial command channel returned nothing while its
video path streamed perfectly. The cause was ModemManager, which had begun probing
the camera's control port as if it were a dial-up modem. One udev rule fixed it.

| | |
|---|---|
| Device | Artosyn Sirius, USB serial `ZBBM5DZFMP`, core serial `YM26012102` |
| Firmware | `DCTOP-640_FW_2025-12-22` |
| Host | Jetson, Ubuntu 22.04, kernel 5.15.148-tegra |
| Protocol | SD-IRM-256-01, 9-byte frames, 115200 8N1 |

Web version of this document (private): <https://claude.ai/code/artifact/ff188094-ad36-4e69-a066-00af670dd54a>

---

## Symptom

Every command went out correctly and nothing came back. Two failure shapes alternated,
seemingly at random.

**Failure A — silence:**

```
$ python3 set_fps_jetson.py 25
port        /dev/serial/by-id/usb-Artosyn_Sirius_ZBBM5DZFMP-if02
setting     25 fps (control code 0x05)
  TX  AA 05 00 05 00 00 B4 EB AA
  RX  (nothing)
FAILED: no reply -- core did not answer within 3s
```

**Failure B — the port is already open:**

```
FAILED  cannot open /dev/serial/...-if02: [Errno 16] Device or resource busy
```

Meanwhile the camera streamed video flawlessly at 32.8 fps throughout. That split —
a healthy data path beside a dead control path on the same USB device — is what made
this hard to place.

---

## What it was not

Each of these was tested and eliminated. Listed so nobody repeats them.

| Hypothesis | Evidence against |
|---|---|
| Wrong baud rate | Silent at all 8 standard rates, 9600 → 921600 |
| Wrong frame or checksum | All 11 frames byte-match the vendor guide's printed values |
| Wrong port or interface | One ttyACM node, correctly bound to `1-2.2:1.2` (class 02) |
| DTR handshake | Silent with DTR asserted and deasserted |
| USB autosuspend | `runtime_suspended_time = 0` — never suspended |
| A faulty individual unit | A second camera behaved identically |
| Needs an active video stream | Answered once while streaming, then failed while streaming |
| Stalled bulk transfer | Draining with `0xC0` got no reply; 30 s passive listen produced 0 bytes |
| Broken USB control path | UVC `SET_CTRL`/`GET_CTRL` over ep0 both worked |

ModemManager was also checked early — and cleared. On 27 Aug its log said it *failed*
to claim the port and gave up. That was true then, and it is why the real cause
survived three days of investigation.

---

## Root cause

On 31 Aug ModemManager's behaviour changed. Instead of declining the port, it
classified it as an AT modem port and started probing:

```
$ journalctl -u ModemManager
11:04:25  could not grab port ttyACM0: unhandled port type
11:08:47  [ttyACM0/at] open blocked by driver for more than 7 seconds!
11:10:29  port ttyACM0 released by device '.../1-2.3'
11:10:30  task 6,ttyACM0: checking support with plugin 'iridium'
```

The `/at` suffix is the whole story. ModemManager opened the control port as an **AT
command port** and wrote modem probe strings into a device that speaks a 9-byte binary
protocol — cycling through plugins (generic, iridium, and others) looking for a modem
that was never there.

That produces both observed failures:

- **Failure B** is direct — while ModemManager holds the port, any other open returns `EBUSY`.
- **Failure A** follows from the probing — AT strings written into a binary command
  parser, and replies consumed by a process that was not us.

**The moment it broke open:** ModemManager released the port at `11:10:29`. The first
read issued seconds later succeeded — the first reply from the core in four days. That
release-then-succeed pairing is the causal link.

---

## The fix

Tell ModemManager to leave the device alone. Two variables, because different
ModemManager versions honour different ones.

`99-thermal-core-mm-ignore.rules` (in this repo):

```udev
# Keep ModemManager away from the SD-IRM-256-01 thermal core's control port.
# Matched on product string as well as VID/PID: 1d6b is the generic Linux
# gadget vendor ID, so VID/PID alone is not specific enough to be safe.
ACTION=="add|change", SUBSYSTEM=="usb", ATTRS{idVendor}=="1d6b", \
  ATTRS{idProduct}=="0101", ATTRS{product}=="Sirius", \
  ENV{ID_MM_DEVICE_IGNORE}="1"

SUBSYSTEM=="tty", ATTRS{idVendor}=="1d6b", ATTRS{idProduct}=="0101", \
  ATTRS{product}=="Sirius", ENV{ID_MM_PORT_IGNORE}="1"
```

Install:

```bash
sudo cp 99-thermal-core-mm-ignore.rules /etc/udev/rules.d/
sudo udevadm control --reload
# then replug the camera
```

The udev attribute walk from `/dev/ttyACM0` was checked to reach
`ATTRS{product}=="Sirius"`, so the match holds from the tty node upward.

---

## Verified after the fix

| Check | Before | After |
|---|---|---|
| Link test `0x08` | no reply ×77 | **answers** |
| Set 25 fps `0x05` | no reply | **`55 05 00 05 00 00 5F EB AA`** |
| Delivered frame rate | 32.80 fps | **25.01 fps** |
| Self-test `0x07` | `0x0001` detector fault | **`0x0000` healthy ×5** |
| Bulk reads | unreachable | serial, firmware, geometry |
| ModemManager touching ttyACM0 | every enumeration | silent |

The frame rate is the point of all this: **25.01 fps delivered**, matching the
commanded rate exactly. At the 50 fps setting the core produces more than the
480 Mbps link can carry and discards frames internally to keep up — invisibly, since
it only numbers the frames it actually sends.

---

## Two things this surfaced

### The detector fault was a false alarm

The first successful self-test, taken while ModemManager was still cycling, returned
`0x0001` — bit 0, detector fault. After the rule was installed it read `0x0000` five
times running. Treat the single fault reading as a casualty of the AT probing, not
evidence of failing hardware. Worth re-checking after any long run, but there is
currently no sign of a detector problem.

### The detector is 640×512, not 1280×1024

With bulk reads working, `0x1C` core info reports the real sensor geometry for the
first time:

```
core info   {"sns_total_width": 640, "sns_total_height": 520,
             "sns_roi_x": 0, "sns_roi_y": 4,
             "sns_roi_width": 640, "sns_roi_height": 512}
firmware    "DCTOP-640_FW_2025-12-22"
```

The UVC stream is 1280×1024, so **the core upscales 2× before sending**. The firmware
name agrees. Three consequences worth acting on:

- Every recording holds **4× the data the detector actually produces** — 43 MB/s of
  which three quarters is interpolation.
- Optical-flow figures are in upscaled pixels. The measured `18.6 px/deg` is `~9.3`
  real detector pixels per degree.
- Any resolution or sensitivity budget built on 1280×1024 needs revisiting.

Worth confirming with the vendor whether a native 640×512 output mode exists — it
would cut the bandwidth problem by 4× and likely remove the frame dropping at 50 fps
entirely.

---

## If it comes back

1. **First** — run `./read_core.py`. It issues the vendor's own link test (`0x08`, read
   core ID) before anything else, so one line tells you whether the channel is up. It
   is read-only and safe to run during a capture.
2. **If EBUSY** — something holds the port. Check `journalctl -u ModemManager -f` for
   `[ttyACM0/at]`. If it appears, the udev rule is missing or was not reloaded.
3. **If silent** — confirm the port exists at all: `ls -l /dev/serial/by-id/`. The
   camera dropped off the bus repeatedly during this investigation; an absent port is a
   cable or power problem, not a protocol one.
4. **Still silent** — power-cycle the core's own supply rather than replugging USB. The
   vendor checklist is explicit that you power the core first, then connect USB.
5. **Never** interleave other commands with a bulk read (`0x12`, `0x1C`, `0x20`). The
   guide warns that nothing may interrupt the sequence; `read_core.py` drives every bulk
   read to its `0` terminator for exactly this reason.

---

## Tools

| File | Purpose |
|---|---|
| `read_core.py` | Reads every documented status value: core ID, temperature, self-test, brightness/contrast/sharpness, serial, firmware, geometry. Read-only. `--selftest` verifies its frames and decoders with no hardware attached. |
| `set_fps_jetson.py` | Switches 25 / 50 fps and times the delivered rate. `--show` works even when the command channel is down, since it only needs the video path. |
| `99-thermal-core-mm-ignore.rules` | The fix above. |

**Why the diagnostic mattered:** the bug was found because `read_core.py` runs the
vendor's link test first and reports one clear failure instead of eleven identical
ones. That made the release-then-succeed transition visible in a single line of output.

---

Protocol reference: SD-IRM-256-01 Serial Configuration Guide — Shanghai Coolcore,
ARS31L SoC. 9-byte frames, 115200 8N1, checksum = `sum(bytes 0..5) & 0xFF`.
