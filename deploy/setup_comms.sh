#!/bin/bash
# Set up this Pi's MAVLink link to the Cube, then prove it works.
#
#   bash deploy/setup_comms.sh              check, install the dialect, test the link
#   bash deploy/setup_comms.sh --check      report only, change nothing
#   bash deploy/setup_comms.sh --no-test    set up, skip talking to the Cube
#
# Four things have to be true before a single bearing can reach the Cube, and
# three of them fail silently. In order of how long they cost to debug:
#
#   1. dtparam=uart0=on in config.txt. On a Pi 5, enable_uart=1 covers only the
#      dedicated debug connector; without uart0=on, GPIO14/15 are not a UART at
#      all and the wiring is irrelevant. Needs a reboot to take effect.
#   2. A regenerated pymavlink dialect. Stock pymavlink cannot encode 42050 and
#      DROPS an incoming 42051 without a word.
#   3. Group membership: the port is root:dialout.
#   4. One reader only. TELEM2 is a single UART; two processes reading it each
#      get a fraction of the bytes and both see CRC errors.
#
# The protocol contract is RPI_COMMS.md. Wiring and bring-up: deploy/COMMS.md.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHECK=0; TEST=1
for a in "$@"; do
  case "$a" in
    --check)   CHECK=1 ;;
    --no-test) TEST=0 ;;
    -h|--help) awk 'NR>1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown option: $a (try --help)" >&2; exit 2 ;;
  esac
done

ok()   { echo "  ok       $*"; }
warn() { echo "  WARNING  $*"; }
bad()  { echo "  FAIL     $*"; }
fail=0

DEV="$(python3 -c "import sys; sys.path.insert(0,'$REPO'); import config; print(config.CUBE_DEVICE)")"
BAUD="$(python3 -c "import sys; sys.path.insert(0,'$REPO'); import config; print(config.CUBE_BAUD)")"

echo "== 1. the UART"
CFG=/boot/firmware/config.txt
[ -f "$CFG" ] || CFG=/boot/config.txt
if grep -qE '^\s*dtparam=uart0=on' "$CFG" 2>/dev/null; then
  ok "dtparam=uart0=on is set in $CFG"
else
  bad "dtparam=uart0=on is MISSING from $CFG. GPIO14/15 are not a UART without it."
  echo "           add it, then reboot:  echo 'dtparam=uart0=on' | sudo tee -a $CFG"
  fail=1
fi
if [ -e "$DEV" ]; then
  ok "$DEV exists ($(stat -c '%U:%G %a' "$DEV"))"
else
  bad "$DEV does not exist. If you just added dtparam=uart0=on, reboot."
  fail=1
fi
# /dev/serial0 is a trap worth naming: it points at the debug connector on some
# Pi 5 configurations, and at the real GPIO UART on others. config.py uses the
# explicit node for that reason; report which one this box has.
if [ -L /dev/serial0 ]; then
  echo "           (/dev/serial0 -> $(readlink /dev/serial0); config.py uses $DEV explicitly, by name, on purpose)"
fi

echo "== 2. permissions"
if id -nG "$(id -un)" | tr ' ' '\n' | grep -qx dialout; then
  ok "user $(id -un) is in group dialout"
else
  bad "user $(id -un) is NOT in group dialout -- opening $DEV will fail"
  echo "           sudo usermod -aG dialout $(id -un)   (then log out and back in)"
  fail=1
fi

echo "== 3. exclusivity"
if command -v fuser >/dev/null 2>&1 && fuser -s "$DEV" 2>/dev/null; then
  warn "$DEV is already held -- one reader only, so stop this first:"
  fuser -v "$DEV" 2>&1 | sed 's/^/           /'
else
  ok "$DEV is free"
fi

echo "== 4. pymavlink dialect"
if python3 -c "import pymavlink" 2>/dev/null; then
  ok "pymavlink $(python3 -c 'import pymavlink; print(pymavlink.__version__)')"
else
  bad "pymavlink is not installed:  pip3 install --user pymavlink"
  fail=1
fi
if [ "$fail" = 0 ]; then
  if python3 "$REPO/comms/gen_dialect.py" --verify >/dev/null 2>&1; then
    ok "dialect thermal_link already carries 42050 + 42051"
  elif [ "$CHECK" = 1 ]; then
    warn "dialect thermal_link is missing or incomplete -- rerun without --check to build it"
  else
    echo "  building the dialect from comms/mavlink/*.msg.xml ..."
    python3 "$REPO/comms/gen_dialect.py" 2>&1 | grep -vE '^(Validation skipped|Parsing|Generating|Merged|Found|MAV_BOOL)' | sed 's/^/  /'
  fi
fi

if [ "$fail" != 0 ]; then
  echo
  echo "setup incomplete -- fix the FAILs above. Nothing was tested."
  exit 1
fi

if [ "$CHECK" = 1 ]; then
  echo; echo "== --check given, nothing changed"; exit 0
fi
if [ "$TEST" = 0 ]; then
  echo; echo "== --no-test given. Verify the link when you are ready:"
  echo "    python3 tools/cube_probe.py"
  exit 0
fi

echo "== 5. talking to the Cube"
echo "  (RPI_COMMS.md section 8 steps 1-7; nothing is sent to guidance)"
if python3 "$REPO/tools/cube_probe.py" --seconds 5; then
  echo
  echo "comms are up. Next:"
  echo "  python3 tools/cube_probe.py --send-detection   step 8, one test bearing (disarmed Cube only)"
  echo "  python3 tools/live_track.py --help             run the pipeline on the live camera with it"
else
  echo
  echo "the probe reported failures above -- deploy/COMMS.md maps each one to its cause."
  exit 1
fi
