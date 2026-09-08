# Device nodes — how the camera and gimbal are named

Three data streams arrive over USB on this Jetson: thermal video, the camera's
serial control channel, and the gimbal's serial telemetry. This document records
which `/dev` node carries which, which name is stable, and which will silently
move on you.

Verified on Jetson1, 31 Aug 2026.

**The short version: never hardcode `video0`, `ttyACM0` or `ttyUSB0`.** They are
assigned in enumeration order and they move. Use the by-id paths below.

---

## Quick reference

| Stream | Use this path | Kernel node today |
|---|---|---|
| Thermal video (capture) | `/dev/v4l/by-id/usb-Artosyn_Sirius_ZBBM5DZFMP-video-index0` | `/dev/video0` |
| Camera serial control | `/dev/serial/by-id/usb-Artosyn_Sirius_ZBBM5DZFMP-if02` | `/dev/ttyACM0` |
| Gimbal serial telemetry | `/dev/local_dds` (or the FTDI by-id path) | `/dev/ttyUSB0` |

```bash
./set_fps_jetson.py --list     # prints the camera's resolved video + serial paths
./calibration/check_gimbal.py              # prints the gimbal's resolved path and link health
```

---

## Physical topology

The camera is **one** USB device exposing six interfaces. The gimbal is a
**separate** device behind a USB-TTL converter.

```
USB bus 001
├── 1-2.3  "Sirius"  (Artosyn thermal core — one device, 6 interfaces)
│   ├── :1.0  class 02  rndis_host   ─┐  USB network gadget
│   ├── :1.1  class 0a  rndis_host   ─┘  (undocumented by the vendor, unused)
│   ├── :1.2  class 02  cdc_acm      ─┐  CONTROL  → /dev/ttyACM0
│   ├── :1.3  class 0a  cdc_acm      ─┘  (the pair is ONE tty, not two)
│   ├── :1.4  class 0e  uvcvideo     ─┐  VIDEO    → /dev/video0 + /dev/video1
│   └── :1.5  class 0e  uvcvideo     ─┘
│
└── 1-2.2  FTDI FT232R  (gimbal USB-TTL converter)
    └── :1.0  ftdi_sio                  GIMBAL   → /dev/ttyUSB0
```

Two things to notice:

- The `cdc_acm` pair (comms class `02` + data class `0a`) forms a **single**
  CDC-ACM function producing **one** tty. It is not two serial ports.
- The two `uvcvideo` interfaces produce two video nodes with different jobs — see
  the video0/video1 section below.
- The RNDIS interfaces are not described in the vendor's serial guide, which
  documents only "a UVC video device and a USB serial port". This firmware build
  exposes more than the document describes.

---

## Why kernel names drift

`videoN`, `ttyACMn` and `ttyUSBn` are handed out in the order devices enumerate.
Observed during one debugging session on 27–31 Aug:

- The camera cycled through USB device numbers **4 → 6 → 9 → 11 → 4 → 24**.
- The camera moved hub port from **`1-2.2` to `1-2.3`**.
- The FTDI converter then took **`1-2.2`** — the slot the camera had occupied.

So a script that hardcodes `/dev/ttyUSB0` for the gimbal can end up addressing a
thermal camera, and vice versa. This is not hypothetical; the ports genuinely
swapped within one session.

---

## The stable names and what creates them

All stable paths are keyed on the device's **USB serial number**, so they survive
re-enumeration.

| Path | Created by | Installed? |
|---|---|---|
| `/dev/v4l/by-id/usb-Artosyn_Sirius_<serial>-video-index0` | `60-persistent-v4l.rules` | stock, `/usr/lib/udev/rules.d/` |
| `/dev/serial/by-id/usb-Artosyn_Sirius_<serial>-if02` | `60-serial.rules` | stock, `/usr/lib/udev/rules.d/` |
| `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_<serial>-if00-port0` | `60-serial.rules` | stock |
| `/dev/local_dds` | `99-drone-ports.rules` | **project**, `/etc/udev/rules.d/` |

Nothing custom is needed for the two `by-id` directories — stock udev provides
them. Only `/dev/local_dds` is ours.

### Reading a by-id name

```
usb-Artosyn_Sirius_ZBBM5DZFMP-if02
    └─vendor─┘ └model┘ └─serial──┘ └── USB interface number
```

The `-if02` suffix is the **USB interface number**, confirmed from udev:

```
/dev/ttyACM0   ID_SERIAL_SHORT=ZBBM5DZFMP   ID_USB_INTERFACE_NUM=02
/dev/ttyUSB0   ID_SERIAL_SHORT=B0033WIE     ID_USB_INTERFACE_NUM=00
```

That is how the camera's control tty is identified: interface `02` is the CDC-ACM
comms interface. The video interfaces are `04` and `05`.

### Project udev rules installed here

`/etc/udev/rules.d/99-drone-ports.rules`:

```udev
# 1. Cube Orange+ (Swarm MAVLink Port)   -- DISABLED (commented out)
####SUBSYSTEM=="tty", ATTRS{idVendor}=="2dae", ATTRS{idProduct}=="1058", \
####  ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="swarm_cube", MODE="0666"

# 2. FTDI Cable (Local DDS/ROS 2 Port)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", \
  SYMLINK+="local_dds", MODE="0666"
```

`/etc/udev/rules.d/99-thermal-core-mm-ignore.rules` — keeps ModemManager off the
camera's control port. See `THERMAL_SERIAL_FAULT.md` for why this is essential.

Note the FTDI rule matches on VID/PID only, with no serial. With two FTDI cables
attached, `local_dds` would point at whichever enumerated last. Add
`ATTRS{serial}=="B0033WIE"` if that ever matters.

---

## video0 vs video1 — the trap

```
/dev/video0   ID_V4L_CAPABILITIES=:capture:   name="Sirius: UVC Camera 0"   ← real
/dev/video1   ID_V4L_CAPABILITIES=:           name="Sirius: UVC Camera 0"   ← metadata
```

**The card names are identical.** Matching on name alone cannot tell them apart.
The only reliable discriminator is `ID_V4L_CAPABILITIES`, which contains
`capture` for the real node and is empty for the metadata node.

`video1` enumerates **no capture formats**, so streaming from it does not fail
fast — it hangs until timeout. That failure looks like a busy device or a broken
camera, and it cost real debugging time.

In the by-id names, `-video-index0` is the capture node and `-video-index1` is the
metadata node. Always use `index0`.

Check which is which:

```bash
udevadm info -q property -n /dev/video0 | grep ID_V4L_CAPABILITIES
v4l2-ctl -d /dev/video0 --list-formats      # capture node lists GREY/NV12/YU12/YUYV
```

---

## Nodes that do NOT exist on this Jetson

| Path | Why it's absent |
|---|---|
| `/dev/thermal0` | Created by the RPi's `99-thermal-cam.rules`, never installed here. This is the sole reason `set_fps.py` cannot measure frame rate on this machine — it hardcodes this path. `set_fps_jetson.py` resolves via `/dev/v4l/by-id/` instead. |
| `/dev/swarm_cube` | The Cube Orange+ rule in `99-drone-ports.rules` is commented out, and no Cube (`2dae:1058`) is attached. |

If you ever port RPi scripts here, `/dev/thermal0` is the first thing to check for.

---

## How the tools resolve nodes

Every tool in this repo resolves by serial through a fallback chain, and never by
kernel name. First existing hit wins.

**Video** (`set_fps_jetson.py`, `record_raw.c`):

```
1. /dev/thermal0                                        (RPi compatibility)
2. /dev/v4l/by-id/usb-Artosyn_Sirius_*-video-index0     (stock udev, preferred here)
3. scan /dev/video* and match card name against "sirius"
   ... then filter on ID_V4L_CAPABILITIES containing "capture"
```

**Camera control** (`set_fps_jetson.py`, `read_core.py`):

```
/dev/serial/by-id/usb-Artosyn_Sirius_*-if02
```

**Gimbal** (`check_gimbal.py`):

```
1. /dev/local_dds
2. /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*-if00-port0
3. /dev/ttyUSB0                                          (last resort)
```

`set_fps_jetson.py` also extracts the serial from the control-port path and uses
it to pick the video node belonging to the **same physical core** — so with two
cameras attached, video and control cannot be mismatched.

---

## Caveat: the camera's serial may not be unique

Two different camera units both reported USB serial **`ZBBM5DZFMP`**. If that is
hardcoded in firmware rather than assigned per unit, then by-id pinning **cannot
distinguish two of these cameras**: only one `by-id` entry appears and the tools
pick unpredictably.

This is fine with a single camera, which is the current setup. Before ever running
two, plug both in and check whether two distinct `by-id` entries appear:

```bash
ls -l /dev/v4l/by-id/ /dev/serial/by-id/
```

The gimbal's FTDI converter does report a real per-unit serial (`B0033WIE`), so it
has no such problem.

---

## Diagnosing node problems

```bash
# What is on the bus?
lsusb
lsusb -t                                   # tree, with driver per interface

# What stable names exist?
ls -l /dev/v4l/by-id/ /dev/serial/by-id/
ls -l /dev/local_dds

# Full udev property dump for one node
udevadm info -q property -n /dev/ttyACM0
udevadm info -a -n /dev/video0 | head -40  # attribute walk, for writing rules

# Which sysfs interface backs a node?
readlink -f /sys/class/tty/ttyACM0/device
readlink -f /sys/class/video4linux/video0/device

# Who holds a serial port?
journalctl -u ModemManager -n 20            # the usual culprit
sudo fuser -v /dev/ttyACM0
```

If a node is missing entirely, check in this order: is the device on the bus
(`lsusb`), did the driver bind (`lsusb -t` shows the driver per interface), and did
udev create the symlink (`ls /dev/serial/by-id/`). Each answers a different
question, and skipping one wastes time.
