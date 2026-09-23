#!/bin/bash
# Provision a freshly flashed Raspberry Pi OS install to run this repo's live
# pipeline exactly as this checkout is configured on the box this was written
# on -- everything outside the git repo that the pipeline depends on: apt/pip
# packages, the GPIO UART, group membership, the external SSD, and the two
# udev rules and one /etc config file that are machine state, not git state.
#
# Run once, as the normal login user (it calls sudo itself), from a fresh
# clone of this repo on the new Pi:
#
#   git clone git@github.com:lat-gaurav/thermal.git ~/thermal
#   cd ~/thermal
#   bash deploy/bootstrap_pi.sh
#
#   bash deploy/bootstrap_pi.sh --check   report only, change nothing
#
# The UART change needs a reboot before it takes effect. Re-run this script
# after rebooting -- everything already done is skipped, and the comms/service
# steps that need the UART will then go through. Idempotent throughout.
#
# What this deliberately does NOT touch: hostname, wifi, ssh, timezone, and
# anything else not needed to run this repo's pipeline.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_USER="${SUDO_USER:-$(id -un)}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"

CHECK=0
for a in "$@"; do
  case "$a" in
    --check)   CHECK=1 ;;
    -h|--help) awk 'NR>1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown option: $a (try --help)" >&2; exit 2 ;;
  esac
done

say()  { echo; echo "== $*"; }
ok()   { echo "  ok        $*"; }
did()  { echo "  DID       $*"; }
would(){ echo "  would do  $*"; }
warn() { echo "  WARNING   $*"; }

run() {  # description, then the command -- honours --check
  local desc="$1"; shift
  if [ "$CHECK" = 1 ]; then would "$desc"; else "$@"; did "$desc"; fi
}

append_once() {  # file line -- idempotent, sudo-writing
  local file="$1" line="$2"
  if [ -f "$file" ] && grep -qxF "$line" "$file" 2>/dev/null; then
    ok "$file already has: $line"
  elif [ "$CHECK" = 1 ]; then
    would "append to $file: $line"
  else
    echo "$line" | sudo tee -a "$file" >/dev/null
    did "appended to $file: $line"
  fi
}

# ------------------------------------------------------------- 1. apt packages --
say "1. apt packages"
APT_PKGS=(
  git build-essential
  python3-pip python3-numpy python3-numpy-dev python3-opencv
  libjpeg-dev v4l-utils
)
MISSING=()
for p in "${APT_PKGS[@]}"; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p")
done
if [ "${#MISSING[@]}" = 0 ]; then
  ok "all present: ${APT_PKGS[*]}"
elif [ "$CHECK" = 1 ]; then
  would "apt-get install ${MISSING[*]}"
else
  sudo apt-get update
  sudo apt-get install -y "${MISSING[@]}"
  did "apt-get install ${MISSING[*]}"
fi

# ------------------------------------------------------------- 2. pip packages --
say "2. pip --user packages (Debian blocks system pip; this repo installs user-site with --break-system-packages, same as the box this was written on)"
for p in pymavlink:pymavlink pyserial:serial; do
  pkg="${p%%:*}"; mod="${p##*:}"
  if python3 -c "import $mod" 2>/dev/null; then
    ok "python3 already imports $mod ($pkg)"
  elif [ "$CHECK" = 1 ]; then
    would "pip3 install --user --break-system-packages $pkg"
  else
    python3 -m pip install --user --break-system-packages "$pkg"
    did "pip3 install --user --break-system-packages $pkg"
  fi
done

# ------------------------------------------------------- 3. bin/record_raw --
say "3. bin/record_raw (the one binary .gitignore excludes -- everything else under bin/ is committed)"
if [ -x "$REPO/bin/record_raw" ]; then
  ok "$REPO/bin/record_raw already built"
elif [ "$CHECK" = 1 ]; then
  would "gcc -O2 -Wall -Wextra -o bin/record_raw tools/record_raw.c"
else
  mkdir -p "$REPO/bin"
  gcc -O2 -Wall -Wextra -o "$REPO/bin/record_raw" "$REPO/tools/record_raw.c"
  did "built $REPO/bin/record_raw"
fi

# --------------------------------------------------------- 4. the GPIO UART --
say "4. GPIO UART for the Cube TELEM2 link (needs a reboot to take effect)"
CFG=/boot/firmware/config.txt
[ -f "$CFG" ] || CFG=/boot/config.txt
if grep -qE '^\s*enable_uart=1' "$CFG" && grep -qE '^\s*dtparam=uart0=on' "$CFG"; then
  ok "enable_uart=1 and dtparam=uart0=on already set in $CFG"
elif [ "$CHECK" = 1 ]; then
  would "add enable_uart=1 / dtparam=uart0=on to $CFG"
else
  {
    echo ""
    echo "[all]"
    grep -qE '^\s*enable_uart=1' "$CFG" || echo "enable_uart=1"
    echo "# Enable RP1 uart0 on GPIO14 (pin 8, TXD) / GPIO15 (pin 10, RXD) for the Cube TELEM2 link."
    grep -qE '^\s*dtparam=uart0=on' "$CFG" || echo "dtparam=uart0=on"
  } | sudo tee -a "$CFG" >/dev/null
  did "added UART config to $CFG -- reboot required before /dev/ttyAMA0 exists"
fi

# -------------------------------------------------------- 5. group membership --
say "5. group membership ($RUN_USER needs dialout for the Cube UART + camera control port, video for /dev/video*)"
for g in dialout video; do
  if id -nG "$RUN_USER" | tr ' ' '\n' | grep -qx "$g"; then
    ok "$RUN_USER already in group $g"
  elif [ "$CHECK" = 1 ]; then
    would "usermod -aG $g $RUN_USER"
  else
    sudo usermod -aG "$g" "$RUN_USER"
    did "usermod -aG $g $RUN_USER -- takes effect on next login"
  fi
done

# --------------------------------------------------- 6. the pixhawk uart rule --
# Not part of this repo (deploy/ only ships the camera rule and the
# ModemManager-ignore rule, installed in step 9 below): pins ttyAMA0/ttyAMA10 to
# group dialout mode 0660, since the serial console previously owned ttyAMA10
# and left it root:tty when its getty was disabled.
say "6. udev: /etc/udev/rules.d/99-pixhawk-uart.rules"
PIXRULE=/etc/udev/rules.d/99-pixhawk-uart.rules
PIXRULE_CONTENT='# TELEM2 link to the Cube. The serial console previously owned ttyAMA10 and left it
# root:tty 0600 when the getty was disabled; pin both UARTs to dialout so master_pipeline
# can open them without running as root.
KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"
KERNEL=="ttyAMA10", GROUP="dialout", MODE="0660"'
if [ -f "$PIXRULE" ] && diff -q <(echo "$PIXRULE_CONTENT") "$PIXRULE" >/dev/null 2>&1; then
  ok "$PIXRULE already up to date"
elif [ "$CHECK" = 1 ]; then
  would "write $PIXRULE"
else
  echo "$PIXRULE_CONTENT" | sudo tee "$PIXRULE" >/dev/null
  sudo udevadm control --reload
  did "wrote $PIXRULE"
fi

# ------------------------------------------------------------- 7. external SSD --
say "7. external SSD (SanDisk 1TB, logs/ and recordings/ symlink into it)"
if [ -d /mnt/external_ssd ]; then ok "/mnt/external_ssd already exists"
else run "mkdir -p /mnt/external_ssd" sudo mkdir -p /mnt/external_ssd; fi
FSTAB_LINE='UUID=7828-48AB  /mnt/external_ssd  exfat  defaults,nofail,uid=1000,gid=1000,umask=022,x-systemd.device-timeout=10s  0  0'
if grep -q '^UUID=7828-48AB' /etc/fstab 2>/dev/null; then
  ok "/etc/fstab already has the SSD entry"
elif [ "$CHECK" = 1 ]; then
  would "append the SSD entry to /etc/fstab"
else
  {
    echo ""
    echo "# SanDisk 1TB Portable SSD -- thermal recordings (~155 GB/hour)."
    echo "# Keyed on UUID, never /dev/sdX: this drive re-enumerated sda -> sdb and broke"
    echo "# a hand-made /dev/sda1 mount. nofail keeps a missing drive from blocking boot."
    echo "$FSTAB_LINE"
  } | sudo tee -a /etc/fstab >/dev/null
  did "appended the SSD entry to /etc/fstab (only takes effect once the drive is plugged in and mounted/rebooted)"
fi
if [ "$CHECK" != 1 ] && mountpoint -q /mnt/external_ssd 2>/dev/null; then
  ok "/mnt/external_ssd already mounted"
elif [ "$CHECK" != 1 ]; then
  sudo mount /mnt/external_ssd 2>/dev/null && did "mounted /mnt/external_ssd" \
    || warn "/mnt/external_ssd not mounted -- plug the SSD in, or it'll mount on next boot (nofail)"
fi
if [ -d "$RUN_HOME/flight-logs" ]; then ok "$RUN_HOME/flight-logs already exists"
else run "mkdir -p $RUN_HOME/flight-logs" sudo -u "$RUN_USER" mkdir -p "$RUN_HOME/flight-logs"; fi

# -------------------------------------------- 8. restore the live tunables file --
# deploy/thermal-live.env in git is the generic template (THERMAL_MODE=web, no
# flight-mode block). What is actually running on this box has been hand-tuned
# past that -- flight mode, THERMAL_LOG_DIR, THERMAL_RECORD, etc. -- and
# deploy/install.sh (step 9) intentionally never overwrites an existing
# /etc/default/thermal-live. So this step writes the box's REAL config first,
# and install.sh will see it already there and leave it alone.
say "8. /etc/default/thermal-live (this box's actual tuning, not the generic template)"
ENVDST=/etc/default/thermal-live
if [ -f "$ENVDST" ]; then
  ok "$ENVDST already present -- left untouched, your edits win"
elif [ "$CHECK" = 1 ]; then
  would "write $ENVDST with this box's current settings (flight mode, SSD paths, etc.)"
else
  sudo install -D -m644 "$REPO/deploy/thermal-live.env" "$ENVDST"
  sudo sed -i \
    -e 's/^THERMAL_MODE=.*/THERMAL_MODE=flight/' \
    -e "s|^THERMAL_LOG_DIR=.*|THERMAL_LOG_DIR=$REPO/logs|" \
    -e 's/^THERMAL_RECORD=.*/THERMAL_RECORD=1/' \
    -e 's/^THERMAL_RAW_RESERVE_MB=.*/THERMAL_RAW_RESERVE_MB=2048/' \
    "$ENVDST"
  did "wrote $ENVDST (template + this box's flight-mode overrides)"
fi

# -------------------------------------------------- 9. dialect + systemd unit --
say "9. mavlink dialect + deploy/install.sh (systemd unit, camera udev rule, ModemManager-ignore rule)"
if [ "$CHECK" = 1 ]; then
  would "python3 comms/gen_dialect.py"
  would "bash deploy/install.sh --check"
else
  if python3 "$REPO/comms/gen_dialect.py" --verify >/dev/null 2>&1; then
    ok "pymavlink dialect already matches comms/mavlink/"
  else
    python3 "$REPO/comms/gen_dialect.py" 2>&1 | grep -vE '^(Validation skipped|Parsing|Generating|Merged|Found|MAV_BOOL)' | sed 's/^/  /'
    did "built the pymavlink dialect"
  fi
  bash "$REPO/deploy/install.sh"
fi

say "done"
if [ "$CHECK" = 1 ]; then
  echo "  --check only -- nothing was changed."
else
  echo "  If step 4 added UART config: reboot now, then re-run this script once more"
  echo "  to pick up /dev/ttyAMA0 (setup_comms.sh's checks, dialect, install.sh preflight)."
  echo "  If step 5 added group membership: log out and back in for it to apply."
  echo "  Not started: sudo systemctl start thermal-live   (see deploy/install.sh's own output above)"
fi
