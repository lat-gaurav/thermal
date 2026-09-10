#!/bin/bash
# Launch the detector/filter pipeline against the LIVE thermal core, and stay
# alive exactly as long as that is actually possible.
#
# This is what thermal-live.service execs. It is a separate script, not an
# ExecStart one-liner, because three things have to happen around the viewer
# and none of them belong inside the viewer itself:
#
#   1. WAIT for the device node. USB enumeration is slower than boot, so at
#      startup /dev/thermal0 legitimately does not exist yet.
#   2. REFUSE to start while something else holds the camera. The core allows
#      exactly one streaming reader; a second one opens and then reads nothing.
#      Failing loudly with the holder's name beats serving a frozen frame.
#   3. WATCHDOG the device node. This is the one that matters: when the camera
#      drops off the USB bus, the viewer's grab thread gets ok=False from
#      cap.read() and sleeps 10 ms in a loop -- forever. It never exits, so
#      Restart=always never fires and the service sits there "active" serving a
#      stale JPEG. The loop below notices the node is gone and kills it, which
#      turns a silent hang into a restart.
#
# Every knob is an environment variable so /etc/default/thermal-live is the
# only file to edit; see that file for what each one is set to and why.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DEVICE="${THERMAL_DEVICE:-/dev/thermal0}"
MODE="${THERMAL_MODE:-web}"
HOST="${THERMAL_HOST:-0.0.0.0}"
PORT="${THERMAL_PORT:-8001}"
WIDTH="${THERMAL_WIDTH:-1280}"
HEIGHT="${THERMAL_HEIGHT:-1024}"
SET_FPS="${THERMAL_SET_FPS-25}"
WAIT_SECS="${THERMAL_WAIT_SECS:-60}"
FB_DETECT="${THERMAL_FB_DETECT:-}"
CUBE_DEVICE="${THERMAL_CUBE_DEVICE:-/dev/ttyAMA0}"
CUBE_BAUD="${THERMAL_CUBE_BAUD:-57600}"
UPLINK="${THERMAL_UPLINK:-1}"
CSV="${THERMAL_TRACK_CSV:-}"
LOG_DIR="${THERMAL_LOG_DIR:-/home/rpi5/thermal/logs}"
LOG_DIR_FALLBACK="${THERMAL_LOG_DIR_FALLBACK:-/home/rpi5/flight-logs}"
REQUIRE_EXTERNAL="${THERMAL_REQUIRE_EXTERNAL:-1}"
RESERVE_MB_FALLBACK="${THERMAL_RAW_RESERVE_MB_FALLBACK:-8192}"
RECORD="${THERMAL_RECORD:-1}"
RESERVE_MB="${THERMAL_RAW_RESERVE_MB:-2048}"
ALWAYS_ON="${THERMAL_ALWAYS_ON:-0}"
DETECTOR="${THERMAL_DETECTOR:-}"
FBDEV="${THERMAL_FBDEV:-/dev/fb0}"
MAX_FPS="${THERMAL_MAX_FPS:-30}"

log() { echo "[run_live] $*"; }

# Filesystem holding the deepest EXISTING ancestor of a path. Walking up matters:
# the target may not exist yet, and df on a missing path says nothing useful.
_fs_of() {
  local p="$1"
  while [ ! -e "$p" ] && [ "$p" != "/" ]; do p="$(dirname "$p")"; done
  df -P "$p" 2>/dev/null | tail -1 | awk '{print $1}'
}

# Decide where this sortie's files go, and NEVER create a directory underneath an
# unmounted mount point.
#
# THE TRAP THIS EXISTS TO AVOID: logs/ is a symlink to /mnt/external_ssd/logs.
# With the SSD unmounted that symlink dangles, /mnt/external_ssd is an ordinary
# empty directory on the SD card, and a plain `mkdir -p` would happily create
# the tree there -- writing ~118 GB/hour into the root filesystem through a path
# that still looks like the SSD. Filling the root filesystem does not just lose
# the recording; it takes the journal, the CSV and very likely the pipeline with
# it. So the primary directory is used only if it really is on a filesystem
# other than root, and otherwise we fall back somewhere honest.
resolve_log_dir() {
  LOG_DIR_USED="$LOG_DIR"
  RESERVE_USED="$RESERVE_MB"
  local root_fs; root_fs="$(_fs_of /)"
  local want_fs; want_fs="$(_fs_of "$LOG_DIR")"
  if [ "$REQUIRE_EXTERNAL" = "1" ] && [ "$want_fs" = "$root_fs" ]; then
    log "WARNING: $LOG_DIR is not on external storage (SSD not mounted?)"
    log "         falling back to $LOG_DIR_FALLBACK on the SD card"
    LOG_DIR_USED="$LOG_DIR_FALLBACK"
    RESERVE_USED="$RESERVE_MB_FALLBACK"
  fi
  mkdir -p "$LOG_DIR_USED" || die "cannot create $LOG_DIR_USED"
  [ -w "$LOG_DIR_USED" ] || die "$LOG_DIR_USED is not writable"
  # Say how long recording can actually last, because on the SD card that is
  # minutes, not hours, and the reserve guard will stop mid-sortie.
  local free_mb; free_mb="$(df -Pm "$LOG_DIR_USED" | tail -1 | awk '{print $4}')"
  local usable_mb=$(( free_mb - RESERVE_USED ))
  if [ "$RECORD" = "1" ]; then
    if [ "$usable_mb" -le 0 ]; then
      log "WARNING: $LOG_DIR_USED has ${free_mb} MB free, under the ${RESERVE_USED} MB"
      log "         reserve -- recording will REFUSE. The pipeline still flies."
    else
      awk -v u="$usable_mb" -v d="$LOG_DIR_USED" -v r="$RESERVE_USED" \
        'BEGIN{printf "[run_live] recording budget: %d MB usable in %s (%.0f MB reserved) = %.1f min at ~1966 MB/min\n", u, d, r, u/1966.0}'
    fi
  fi
}
die() { echo "[run_live] FATAL: $*" >&2; exit 1; }

# --- 1. the device node ------------------------------------------------------
if [ ! -e "$DEVICE" ]; then
  log "waiting up to ${WAIT_SECS}s for $DEVICE"
  for _ in $(seq 1 "$WAIT_SECS"); do
    [ -e "$DEVICE" ] && break
    sleep 1
  done
fi
[ -e "$DEVICE" ] || die "$DEVICE never appeared after ${WAIT_SECS}s. Is the core plugged in? ls -l /dev/serial/by-id/"

# --- 2. exclusivity ----------------------------------------------------------
# fuser only reports processes this user can see, so a holder running as another
# user shows up as "free" here and the viewer's own open() is what fails. That is
# fine -- this check exists to turn the common case into a readable message, not
# to be authoritative.
if command -v fuser >/dev/null 2>&1; then
  log "waiting up to ${WAIT_SECS}s for $DEVICE to be free"
  for _ in $(seq 1 "$WAIT_SECS"); do
    fuser -s "$DEVICE" 2>/dev/null || break
    sleep 1
  done
  if fuser -s "$DEVICE" 2>/dev/null; then
    fuser -v "$DEVICE" 2>&1 | sed 's/^/[run_live]   /' >&2
    die "$DEVICE is held by the process above. The core allows one reader; stop it first."
  fi
fi

# --- 2b. the Cube's UART, in track mode only --------------------------------
# TELEM2 is a single UART and admits one reader, exactly like the camera: two
# processes reading it each get a fraction of the bytes and both see CRC errors.
# Same wait-then-refuse as above, for the same reason.
if [ "$MODE" = "track" ] || [ "$MODE" = "flight" ]; then
  [ -e "$CUBE_DEVICE" ] || die "MODE=$MODE needs the Cube but $CUBE_DEVICE does not exist. Is dtparam=uart0=on set in config.txt (and rebooted since)? See deploy/COMMS.md."
  if command -v fuser >/dev/null 2>&1; then
    for _ in $(seq 1 "$WAIT_SECS"); do
      fuser -s "$CUBE_DEVICE" 2>/dev/null || break
      sleep 1
    done
    if fuser -s "$CUBE_DEVICE" 2>/dev/null; then
      fuser -v "$CUBE_DEVICE" 2>&1 | sed 's/^/[run_live]   /' >&2
      die "$CUBE_DEVICE is held by the process above. TELEM2 allows one reader; stop it first."
    fi
  fi
  log "cube link $CUBE_DEVICE @ $CUBE_BAUD, uplink=$UPLINK"
fi

# --- 3. frame rate ----------------------------------------------------------
# The core comes up at 50 fps on every power cycle and does not persist the
# setting, so this has to be re-applied per start rather than once on a bench.
# 25 is the rate the USB path actually honours end to end (tools/set_fps.py
# documents the measurement: mode 50 delivers 30.17 fps, mode 25 delivers 25.04).
# Best-effort on purpose: a hiccup on the control port must never stop the
# pipeline from running, it just runs at whatever the core is already doing.
if [ -n "$SET_FPS" ]; then
  if compgen -G "/dev/serial/by-id/usb-Artosyn_Sirius_*-if02" >/dev/null; then
    log "setting core to ${SET_FPS} fps (best effort)"
    /usr/bin/python3 "$REPO/tools/set_fps.py" "$SET_FPS" --force \
      2>&1 | sed 's/^/[run_live]   /' || log "set_fps failed -- continuing at the core's current rate"
  else
    log "no core control port under /dev/serial/by-id -- skipping the fps set"
  fi
fi

# --- 4. run, and supervise ---------------------------------------------------
case "$MODE" in
  web)
    ARGS=("$REPO/tools/web_viewer.py" --live "$DEVICE"
          --width "$WIDTH" --height "$HEIGHT" --host "$HOST" --port "$PORT")
    log "web viewer on http://${HOST}:${PORT}/  device=$DEVICE ${WIDTH}x${HEIGHT}"
    ;;
  fb)
    [ -e "$FBDEV" ] || die "MODE=fb but $FBDEV does not exist (no HDMI display attached, or the console is not on KMS)"
    ARGS=("$REPO/tools/fb_viewer.py" --live "$DEVICE"
          --width "$WIDTH" --height "$HEIGHT" --fbdev "$FBDEV" --max-fps "$MAX_FPS")
    [ -n "$FB_DETECT" ] && ARGS+=(--detect "$FB_DETECT")
    log "framebuffer viewer on $FBDEV  device=$DEVICE ${WIDTH}x${HEIGHT}"
    ;;
  track)
    ARGS=("$REPO/tools/live_track.py" --device "$DEVICE"
          --width "$WIDTH" --height "$HEIGHT"
          --cube "$CUBE_DEVICE" --baud "$CUBE_BAUD" --quiet)
    [ "$UPLINK" = "1" ] || ARGS+=(--no-uplink)
    [ -n "$CSV" ] && ARGS+=(--csv "$CSV")
    log "live tracker: camera=$DEVICE cube=$CUBE_DEVICE uplink=$UPLINK"
    ;;
  flight)
    # ONE timestamp for the whole sortie, computed here and shared by the CSV and
    # every .rawrec episode, so the pair is unambiguous without guessing. It
    # carries BOTH a wall clock and the boot_id, and needs both: the date so a
    # file can be matched to a sortie by eye and files sort chronologically, and
    # the boot_id because the Pi 5 only keeps time across power-off with its RTC
    # battery fitted and there is no network in the field to re-sync from. If the
    # clock comes up wrong, every sortie that day would otherwise land on the
    # same name and overwrite the last.
    S="$(date +%Y%m%d-%H%M%S)-$(cut -c1-8 /proc/sys/kernel/random/boot_id)"
    resolve_log_dir
    ARGS=("$REPO/tools/flight_pipeline.py" --camera "$DEVICE"
          --width "$WIDTH" --height "$HEIGHT"
          --cube "$CUBE_DEVICE" --baud "$CUBE_BAUD"
          --out-csv "$LOG_DIR_USED/los-${S}.csv"
          --raw-reserve-mb "$RESERVE_USED")
    [ "$RECORD" = "1" ] && ARGS+=(--raw-video "$LOG_DIR_USED/flight-${S}.rawrec")
    [ "$UPLINK" = "1" ] || ARGS+=(--no-uplink)
    [ "$ALWAYS_ON" = "1" ] && ARGS+=(--always-on)
    [ -n "$DETECTOR" ] && ARGS+=(--detector "$DETECTOR")
    log "flight pipeline: stamp $S  log_dir $LOG_DIR_USED  record=$RECORD uplink=$UPLINK always_on=$ALWAYS_ON"
    ;;
  *)
    die "THERMAL_MODE=$MODE is not one of: web, fb, track, flight"
    ;;
esac

# -u so the journal gets output as it happens rather than in 4 KB blocks.
/usr/bin/python3 -u "${ARGS[@]}" &
CHILD=$!

# systemd sends SIGTERM to the whole cgroup, but be explicit: the viewer needs to
# run its own cleanup (drain the grab thread, release the camera, repaint the
# console in fb mode) instead of being torn down under us.
trap 'log "signal received, stopping viewer"; kill -TERM "$CHILD" 2>/dev/null; wait "$CHILD" 2>/dev/null; exit 0' TERM INT

while true; do
  if ! kill -0 "$CHILD" 2>/dev/null; then
    wait "$CHILD"; rc=$?
    log "viewer exited with status $rc"
    exit "$rc"
  fi
  if [ ! -e "$DEVICE" ]; then
    log "$DEVICE vanished -- killing the child so systemd restarts and waits for it"
    kill -TERM "$CHILD" 2>/dev/null
    wait "$CHILD" 2>/dev/null
    exit 1
  fi
  if { [ "$MODE" = "track" ] || [ "$MODE" = "flight" ]; } && [ ! -e "$CUBE_DEVICE" ]; then
    log "$CUBE_DEVICE vanished -- killing the child so systemd restarts and waits for it"
    kill -TERM "$CHILD" 2>/dev/null
    wait "$CHILD" 2>/dev/null
    exit 1
  fi
  sleep 2
done
