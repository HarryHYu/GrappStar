#!/usr/bin/env bash
# GrappStar Jetson install. Run from anywhere, as your normal user (not with sudo):
#   bash setup/install.sh              install packages, groups, udev rules
#   bash setup/install.sh --service    the same, plus start the operator page at boot
# Options:
#   --service        install and enable the systemd service (grappstar.service)
#   --no-apt         skip apt (no internet, or packages already there)
#   --remove-brltty  remove brltty (it grabs CH340 serial ports on Ubuntu 22.04)
#   --base-serial S  the base DevKitC's USB serial number for /dev/grappstar-base
#                    (without it, plug in ONLY the base by USB and it offers to find it)
#   -h, --help       show this help
# Safe to run again: every step checks first.
set -euo pipefail

SERVICE=0
DO_APT=1
REMOVE_BRLTTY=0
BASE_SERIAL=""
while [ $# -gt 0 ]; do
    case "$1" in
        --service) SERVICE=1 ;;
        --no-apt) DO_APT=0 ;;
        --remove-brltty) REMOVE_BRLTTY=1 ;;
        --base-serial)
            if [ $# -lt 2 ]; then echo "--base-serial needs a value" >&2; exit 2; fi
            BASE_SERIAL="$2"; shift ;;
        --base-serial=*) BASE_SERIAL="${1#--base-serial=}" ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
    shift
done
PLACEHOLDER="REPLACE_WITH_BASE_SERIAL"
valid_serial() { [[ "$1" =~ ^[A-Za-z0-9:._-]+$ ]] && [ "$1" != "$PLACEHOLDER" ]; }
if [ -n "$BASE_SERIAL" ] && ! valid_serial "$BASE_SERIAL"; then
    echo "--base-serial: '$BASE_SERIAL' does not look like a serial number (letters, digits, : . _ - only)" >&2
    exit 2
fi

SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON_DIR="$(dirname "$SETUP_DIR")"
RUN_USER="$(id -un)"

step() { printf '\n== %s\n' "$*"; }
ok()   { printf '   ok: %s\n' "$*"; }
warn() { printf '   WARNING: %s\n' "$*"; }

if [ "$(id -u)" -eq 0 ]; then
    echo "Run this as your normal user, not with sudo. It asks for your password when it needs it." >&2
    exit 1
fi

step "Checking the machine"
if [ -f /etc/nv_tegra_release ]; then
    ok "Jetson: $(head -n 1 /etc/nv_tegra_release)"
else
    warn "this does not look like a Jetson. Carrying on (fine on an Ubuntu PC for testing)."
fi
PYVER="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
ok "python3 is $PYVER (JetPack 6.2 ships 3.10)"
ok "code folder: $JETSON_DIR"

# ---------------------------------------------------------------------------
if [ "$DO_APT" -eq 1 ]; then
    step "Installing system packages with apt (snap is broken on JetPack 6.2, so apt only)"
    sudo apt-get update
    # numpy from apt is 1.21.x: below 2, which is what we want.
    sudo apt-get install -y python3-pip python3-numpy python3-serial python3-pytest v4l-utils network-manager
    # OpenCV: JetPack usually already has cv2 for Python (NVIDIA build, with GStreamer).
    # Only install Ubuntu's python3-opencv if cv2 is missing, so the two do not clash.
    if python3 -c 'import cv2' >/dev/null 2>&1; then
        ok "OpenCV already installed: $(python3 -c 'import cv2; print(cv2.__version__)'), not touching it"
    else
        sudo apt-get install -y python3-opencv
    fi
else
    step "Skipping apt (--no-apt)"
fi

# ---------------------------------------------------------------------------
step "Installing anything still missing with pip (--user, numpy kept below 2)"
if python3 -m pip --version >/dev/null 2>&1; then
    # requirements.txt pins numpy>=1.21,<2, so pip cannot upgrade it to 2.x.
    python3 -m pip install --user -r "$JETSON_DIR/requirements.txt"
else
    warn "pip not found. Install it with: sudo apt-get install python3-pip"
fi
NUMPY_VER="$(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo missing)"
case "$NUMPY_VER" in
    missing) warn "numpy is not importable" ;;
    2.*) warn "numpy is $NUMPY_VER. It must be below 2. Fix: python3 -m pip install --user 'numpy>=1.21,<2'" ;;
    *) ok "numpy $NUMPY_VER" ;;
esac

# ---------------------------------------------------------------------------
step "Adding $RUN_USER to groups dialout (serial) and video (cameras)"
NEED_RELOGIN=0
for grp in dialout video; do
    if [[ " $(id -nG "$RUN_USER") " == *" $grp "* ]]; then
        ok "already in $grp"
    else
        sudo usermod -aG "$grp" "$RUN_USER"
        ok "added to $grp"
        NEED_RELOGIN=1
    fi
done

# ---------------------------------------------------------------------------
step "Installing udev rules (/dev/grappstar-base, /dev/grappstar-rovercam, /dev/grappstar-dartcam)"
RULES_DST=/etc/udev/rules.d/99-grappstar.rules

# The base and the dart's XIAO are both 303a:1001, so the base is matched by its USB serial number.
# Order: --base-serial, then the serial already in the installed rule, then offer the one board plugged in.
SERIAL_SRC=""
if [ -n "$BASE_SERIAL" ]; then
    SERIAL_SRC="from --base-serial"
elif [ -f "$RULES_DST" ]; then
    old="$(grep -o 'ATTRS{serial}=="[^"]*", SYMLINK+="grappstar-base"' "$RULES_DST" | head -n 1 | cut -d'"' -f2 || true)"
    if [ -n "$old" ] && valid_serial "$old"; then
        BASE_SERIAL="$old"
        SERIAL_SRC="kept from the installed rule"
    fi
fi
# List every 303a:1001 serial port (ESP32-S3 boards) with its serial number.
CANDIDATES=()
for dev in /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    props="$(udevadm info -q property -n "$dev" 2>/dev/null || true)"
    if grep -qx 'ID_VENDOR_ID=303a' <<<"$props" && grep -qx 'ID_MODEL_ID=1001' <<<"$props"; then
        ser="$(grep '^ID_SERIAL_SHORT=' <<<"$props" | cut -d= -f2- || true)"
        if [ -n "$ser" ]; then
            CANDIDATES+=("$ser")
            ok "ESP32-S3 board on $dev, serial $ser"
        fi
    fi
done
if [ -z "$BASE_SERIAL" ]; then
    if [ "${#CANDIDATES[@]}" -eq 1 ] && valid_serial "${CANDIDATES[0]}"; then
        echo "   Found one ESP32-S3 board (303a:1001), serial ${CANDIDATES[0]}."
        echo "   Use it as the BASE only if the base is the ONLY board plugged in by USB (the dart unplugged)."
        ans=""
        if [ -t 0 ]; then
            read -r -p "   Is only the base plugged in? Write this serial into the rule? [y/N] " ans || ans=""
        fi
        if [[ "$ans" =~ ^[Yy] ]]; then
            BASE_SERIAL="${CANDIDATES[0]}"
            SERIAL_SRC="found on USB"
        fi
    elif [ "${#CANDIDATES[@]}" -gt 1 ]; then
        warn "more than one ESP32-S3 board is plugged in (base and dart look the same). Unplug the dart, keep only"
        warn "the base plugged in by USB and run this again, or pass --base-serial SERIAL."
    fi
fi
tmp_rules="$(mktemp)"
if [ -n "$BASE_SERIAL" ]; then
    sed -e "s|$PLACEHOLDER|$BASE_SERIAL|" "$SETUP_DIR/99-grappstar.rules" > "$tmp_rules"
else
    cp "$SETUP_DIR/99-grappstar.rules" "$tmp_rules"
fi
sudo install -m 0644 "$tmp_rules" "$RULES_DST"
rm -f "$tmp_rules"
sudo udevadm control --reload-rules
sudo udevadm trigger
ok "rules installed. Unplug and replug USB devices if the names do not appear."
if [ -n "$BASE_SERIAL" ]; then
    ok "base serial $BASE_SERIAL ($SERIAL_SRC): /dev/grappstar-base points at that board only"
else
    warn "base serial not set, so /dev/grappstar-base will not appear. Battery unplugged, plug in ONLY the base"
    warn "by USB (dart unplugged), then run: bash setup/install.sh --no-apt   (README step 4)"
fi

# ---------------------------------------------------------------------------
if dpkg -s brltty >/dev/null 2>&1; then
    if [ "$REMOVE_BRLTTY" -eq 1 ]; then
        step "Removing brltty"
        sudo apt-get remove -y brltty
    else
        warn "brltty is installed. It can steal CH340 serial ports. If /dev/ttyUSB0 vanishes, run: bash setup/install.sh --remove-brltty"
    fi
fi

# ---------------------------------------------------------------------------
if [ "$SERVICE" -eq 1 ]; then
    step "Installing the systemd service (operator page starts at boot)"
    tmp="$(mktemp)"
    trap 'rm -f "$tmp"' EXIT
    sed -e "s|@USER@|$RUN_USER|g" -e "s|@JETSON_DIR@|$JETSON_DIR|g" "$SETUP_DIR/grappstar.service" > "$tmp"
    sudo install -m 0644 "$tmp" /etc/systemd/system/grappstar.service
    sudo systemctl daemon-reload
    sudo systemctl enable grappstar.service
    sudo systemctl restart grappstar.service
    ok "service enabled. Check it with: systemctl status grappstar"
else
    step "Skipping the systemd service (add --service to install it)"
fi

# ---------------------------------------------------------------------------
step "Environment check"
python3 "$SETUP_DIR/check_env.py" || warn "check_env.py reported a problem (see above)"

step "Done"
if [ "$NEED_RELOGIN" -eq 1 ]; then
    echo "   IMPORTANT: log out and back in (or reboot) so the new groups take effect."
fi
echo "   Next: README.md step 3 (test with no hardware)."
