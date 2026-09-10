#!/bin/bash
# Deploy this checkout's live pipeline as a systemd service.
#
#   bash deploy/install.sh              install/update, do not start
#   bash deploy/install.sh --start      ... and start it now
#   bash deploy/install.sh --enable     ... and start it at boot too
#   bash deploy/install.sh --check      preflight only, write nothing
#   bash deploy/install.sh --uninstall  remove what this installed
#
# Everything installed comes from this repository and points back into it; the
# unit is rendered from deploy/thermal-live.service with @REPO@/@USER@ filled in,
# so the service runs THIS checkout wherever it happens to live.
#
# It does not start the service by default. Starting it takes the camera, which
# only permits one reader, so that is a decision for whoever is at the rig --
# not a side effect of installing a file.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO/deploy"
RUN_USER="${SUDO_USER:-$(id -un)}"
STAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP="$SRC/backups/$STAMP"

UNIT=/etc/systemd/system/thermal-live.service
DROPIN=/etc/systemd/system/thermal-live.service.d/10-flight.conf
ENVDST=/etc/default/thermal-live
RULES=/etc/udev/rules.d/99-thermal-core-mm-ignore.rules
CAMRULE=/etc/udev/rules.d/99-thermal-core.rules

START=0; ENABLE=0; CHECK=0; UNINSTALL=0
for a in "$@"; do
  case "$a" in
    --start)     START=1 ;;
    --enable)    START=1; ENABLE=1 ;;
    --check)     CHECK=1 ;;
    --uninstall) UNINSTALL=1 ;;
    -h|--help)   awk 'NR>1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)           echo "unknown option: $a (try --help)" >&2; exit 2 ;;
  esac
done

say()  { echo "$*"; }
ok()   { echo "  ok       $*"; }
warn() { echo "  WARNING  $*"; }
bad()  { echo "  FAIL     $*"; }

# ---------------------------------------------------------------- uninstall --
if [ "$UNINSTALL" = 1 ]; then
  say "== removing the service"
  sudo systemctl disable --now thermal-live.service 2>/dev/null || true
  for f in "$UNIT" "$DROPIN"; do
    [ -e "$f" ] && { sudo rm -f "$f"; say "  removed  $f"; } || say "  absent   $f"
  done
  say "  kept     $RULES  (a ModemManager guard, harmless and shared -- remove by hand if you really want it gone)"
  say "  kept     $CAMRULE  (creates /dev/thermal0; removing it would break every tool here, not just the service)"
  sudo systemctl daemon-reload
  say "done. Nothing in $REPO was touched."
  exit 0
fi

# ----------------------------------------------------------------- preflight --
# Every check that would otherwise turn into a crash-loop at 05:00 in a field.
say "== preflight"
fail=0

for f in tools/web_viewer.py tools/fb_viewer.py tools/rawrec_viewer.py tools/set_fps.py \
         tools/live_track.py tools/cube_probe.py config.py detector/tophat_scr.py \
         filters/clutter_reject.py filters/los_proximity.py \
         initialisation/rightmost_isolated.py experiment/los_static_track.py \
         comms/dialect.py comms/bearing.py comms/cube_link.py comms/gen_dialect.py \
         comms/mavlink/detection_target_data.msg.xml \
         comms/mavlink/gcs_target_bearing.msg.xml \
         deploy/99-thermal-core.rules deploy/run_live.sh deploy/setup_comms.sh \
         deploy/10-flight.conf flight/rawrec.py flight/rc_arm.py flight/camera.py \
         tools/flight_pipeline.py; do
  if [ -f "$REPO/$f" ]; then ok "$f"; else bad "MISSING $REPO/$f"; fail=1; fi
done

if python3 -c 'import cv2, numpy' 2>/dev/null; then
  ok "python3 imports cv2 + numpy ($(python3 -c 'import cv2,numpy;print("cv2 "+cv2.__version__+", numpy "+numpy.__version__)'))"
else
  bad "python3 cannot import cv2 and/or numpy -- the viewer will not start"; fail=1
fi

# The pipeline's own import graph, exactly as the service loads it: the loaders
# walk detector/, filters/ and initialisation/ and exec each module. A syntax
# error, or a config.py key a module reads that does not exist, shows up here
# rather than 5 s after systemctl start.
if PIPE="$(python3 - "$REPO" <<'PYPROBE' 2>&1
import sys, pathlib, importlib.util
repo = pathlib.Path(sys.argv[1])
sys.path.insert(0, str(repo / "tools"))
spec = importlib.util.spec_from_file_location("rawrec_viewer", repo / "tools" / "rawrec_viewer.py")
rv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rv)
d = [n for n, _ in rv.load_detectors()]
f = [n for n, _ in rv.load_filters()]
i = [n for n, _ in rv.load_initialisers()]
assert d, "no detector module loaded from detector/"
print("detectors=%s  filters=%s  initialisers=%s" % (",".join(d), ",".join(f), ",".join(i)))
PYPROBE
)"; then
  ok "pipeline modules load: $PIPE"
else
  bad "the detector/filter/initialiser modules do not import:"
  echo "$PIPE" | sed 's/^/           /'
  fail=1
fi

DEV="$(sed -n 's/^THERMAL_DEVICE=//p' "$SRC/thermal-live.env" | tail -1)"; DEV="${DEV:-/dev/thermal0}"
if [ -e "$DEV" ]; then
  ok "$DEV present -> $(readlink -f "$DEV")"
  if command -v fuser >/dev/null 2>&1 && fuser -s "$DEV" 2>/dev/null; then
    warn "$DEV is currently held by another process -- the service will wait for it, not fight it:"
    fuser -v "$DEV" 2>&1 | sed 's/^/           /'
  else
    ok "$DEV is free"
  fi
else
  warn "$DEV is absent (camera unplugged?). Installing anyway; run_live.sh waits for it."
fi

MODE="$(sed -n 's/^THERMAL_MODE=//p' "$SRC/thermal-live.env" | tail -1)"; MODE="${MODE:-web}"
ok "THERMAL_MODE=$MODE"

# The Cube link only has to work in track mode. Reported either way, because a
# broken link discovered on the flight line is worse than a warning here.
CUBEDEV="$(sed -n 's/^THERMAL_CUBE_DEVICE=//p' "$SRC/thermal-live.env" | tail -1)"; CUBEDEV="${CUBEDEV:-/dev/ttyAMA0}"
if [ "$MODE" = "track" ]; then comms_bad=bad; else comms_bad=warn; fi
if python3 "$REPO/comms/gen_dialect.py" --verify >/dev/null 2>&1; then
  ok "mavlink dialect carries 42050 + 42051"
else
  $comms_bad "mavlink dialect is missing 42050/42051 -- run: bash deploy/setup_comms.sh"
  [ "$MODE" = "track" ] && fail=1
fi
if [ -e "$CUBEDEV" ]; then
  ok "$CUBEDEV present"
  if command -v fuser >/dev/null 2>&1 && fuser -s "$CUBEDEV" 2>/dev/null; then
    warn "$CUBEDEV is held by another process -- TELEM2 allows one reader:"
    fuser -v "$CUBEDEV" 2>&1 | sed 's/^/           /'
  fi
else
  $comms_bad "$CUBEDEV absent (dtparam=uart0=on in config.txt, then reboot)"
  [ "$MODE" = "track" ] && fail=1
fi
if id -nG "$RUN_USER" | tr ' ' '\n' | grep -qx dialout; then
  ok "user $RUN_USER is in group dialout"
else
  warn "user $RUN_USER is not in group dialout; the unit grants it via SupplementaryGroups"
fi

PORT="$(sed -n 's/^THERMAL_PORT=//p' "$SRC/thermal-live.env" | tail -1)"; PORT="${PORT:-8001}"
if [ "$MODE" = "web" ]; then
  if command -v ss >/dev/null 2>&1 && ss -lnt 2>/dev/null | grep -q ":${PORT} "; then
    warn "TCP port $PORT is already in use -- change THERMAL_PORT in $ENVDST"
  else
    ok "TCP port $PORT is free"
  fi
fi

if id -nG "$RUN_USER" | tr ' ' '\n' | grep -qx video; then
  ok "user $RUN_USER is in group video"
else
  warn "user $RUN_USER is not in group video; the unit grants it via SupplementaryGroups, but shell testing will fail"
fi

[ "$fail" = 0 ] || { echo; echo "preflight failed -- nothing installed."; exit 1; }
if [ "$CHECK" = 1 ]; then echo; say "== --check given, nothing written"; exit 0; fi

# ------------------------------------------------------------------- install --
say ""
say "== backing up current copies to $BACKUP"
mkdir -p "$BACKUP"
for d in "$UNIT" "$DROPIN" "$ENVDST" "$RULES" "$CAMRULE"; do
  [ -f "$d" ] && cp -p "$d" "$BACKUP/$(basename "$d")" && say "  saved    $d"
done

# The unit is a template: fill in this checkout's path and the invoking user, so
# a service installed from /home/rpi5/thermal never silently runs some other copy.
RENDERED="$(mktemp)"; trap 'rm -f "$RENDERED"' EXIT
sed -e "s|@REPO@|$REPO|g" -e "s|@USER@|$RUN_USER|g" "$SRC/thermal-live.service" > "$RENDERED"

say "== installing"
install_if_changed() {  # src dst
  if cmp -s "$1" "$2" 2>/dev/null; then say "  unchanged $2"
  else sudo install -D -m644 "$1" "$2"; say "  INSTALLED $2"; fi
}
install_if_changed "$RENDERED" "$UNIT"
install_if_changed "$SRC/10-flight.conf" "$DROPIN"
# The env file is the tuning surface: never clobber edits made on the box.
if [ -f "$ENVDST" ]; then
  say "  KEPT      $ENVDST (already present -- your edits win; reference copy: $SRC/thermal-live.env)"
else
  install_if_changed "$SRC/thermal-live.env" "$ENVDST"
fi
# Ships in this repo's root, and README.md says it belongs in /etc/udev/rules.d:
# it stops ModemManager AT-probing the core's CDC-ACM port, which once took the
# serial channel out for four days (docs/THERMAL_SERIAL_FAULT.md).
install_if_changed "$REPO/99-thermal-core-mm-ignore.rules" "$RULES"
# This repo's own /dev/thermal0 symlink, so the node does not depend on any
# other project's udev file (which also used to auto-start that project).
install_if_changed "$SRC/99-thermal-core.rules" "$CAMRULE"
chmod +x "$SRC/run_live.sh"

say "== reloading systemd and udev"
sudo systemctl daemon-reload
sudo udevadm control --reload
systemd-analyze verify thermal-live.service 2>&1 | sed 's/^/  /' || say "  (systemd-analyze reported the above)"

say ""
say "== effective configuration"
systemctl cat thermal-live 2>/dev/null | grep -E '^(WorkingDirectory|ExecStart|User)=' | sed 's/^/  /'
grep -E '^THERMAL_' "$ENVDST" | sed 's/^/  /'

say ""
if [ "$ENABLE" = 1 ]; then
  say "== enabling at boot and starting"
  sudo systemctl enable --now thermal-live.service
elif [ "$START" = 1 ]; then
  say "== starting (not enabled at boot)"
  sudo systemctl start thermal-live.service
else
  say "NOT started. Starting it takes the camera, which allows one reader only:"
  say "    sudo systemctl start thermal-live      # this session only"
  say "    sudo systemctl enable --now thermal-live   # and at every boot"
  say "Backup of the previous /etc copies: $BACKUP"
  exit 0
fi

sleep 4
systemctl --no-pager --lines=20 status thermal-live.service || true
say ""
say "  view:  http://$(hostname -I 2>/dev/null | awk '{print $1}'):${PORT}/"
say "  logs:  journalctl -u thermal-live -f"
say "  stop:  sudo systemctl stop thermal-live"
