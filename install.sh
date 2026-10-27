#!/bin/bash
# install.sh - Installer for HP Elite x2 1012 G2 Camera & IR System
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Error: This script must be run as root (e.g. sudo ./install.sh)" >&2
  exit 1
fi

BOARD=$(cat /sys/class/dmi/id/board_name 2>/dev/null || echo "UNKNOWN")
if [ "$BOARD" != "82CA" ]; then
  echo "Warning: Motherboard '$BOARD' detected. This package is designed for HP Elite x2 1012 G2 (board 82CA)."
  read -p "Proceed anyway? (y/N): " -r CONFIRM
  [[ "$CONFIRM" =~ ^[Yy]$ ]] || exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_NAME="hp-camera-unified"
PACKAGE_VERSION="1.0.0"
SRC_DEST="/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}"

echo "=== 1. Installing DKMS Kernel Driver Package ==="
mkdir -p "$SRC_DEST"
cp -a "$SCRIPT_DIR"/dkms/* "$SRC_DEST/"

if dkms status | grep -q "${PACKAGE_NAME}/${PACKAGE_VERSION}"; then
  echo "Removing previous ${PACKAGE_NAME}/${PACKAGE_VERSION} registration..."
  dkms remove "${PACKAGE_NAME}/${PACKAGE_VERSION}" --all || true
fi

dkms add "${PACKAGE_NAME}/${PACKAGE_VERSION}"
dkms build "${PACKAGE_NAME}/${PACKAGE_VERSION}"
dkms install "${PACKAGE_NAME}/${PACKAGE_VERSION}"

echo "=== 2. Installing Modprobe & WirePlumber Configurations ==="
install -m 644 "$SCRIPT_DIR/configs/hp-camera-unified.conf" /etc/modprobe.d/hp-camera-unified.conf
mkdir -p /etc/wireplumber/wireplumber.conf.d/
install -m 644 "$SCRIPT_DIR/configs/50-disable-hp-ir-camera.conf" /etc/wireplumber/wireplumber.conf.d/50-disable-hp-ir-camera.conf

echo "=== 3. Compiling and Installing On-Demand IR Feeder ==="
make -C "$SCRIPT_DIR/feeder" clean
make -C "$SCRIPT_DIR/feeder"
install -m 755 "$SCRIPT_DIR/feeder/hp-ir-feeder" /usr/local/bin/hp-ir-feeder

echo "=== 4. Installing Systemd Services and Helper Scripts ==="
mkdir -p /usr/local/libexec/
install -m 755 "$SCRIPT_DIR/systemd/hp-camera-unified.sh" /usr/local/libexec/hp-camera-unified
install -m 644 "$SCRIPT_DIR/systemd/hp-camera-unified.service" /etc/systemd/system/hp-camera-unified.service
install -m 644 "$SCRIPT_DIR/feeder/hp-ir-feeder.service" /etc/systemd/system/hp-ir-feeder.service

systemctl daemon-reload
systemctl enable hp-camera-unified.service
systemctl enable hp-ir-feeder.service

echo "=== 5. Starting Camera Services ==="
systemctl restart hp-camera-unified.service
systemctl restart hp-ir-feeder.service

# Restart active user WirePlumber session if available
for uid in $(loginctl list-users --no-legend 2>/dev/null | awk '{print $1}'); do
  systemctl --user -M "${uid}@" restart wireplumber 2>/dev/null || true
done

echo ""
echo "=========================================================="
echo " Installation completed successfully!"
echo " - Front Webcam:  OV5670 (5MP RGB) -> Default in Snapshot/Apps"
echo " - Rear Webcam:   OV8858 (8MP RGB) -> Toggle via Camera Apps"
echo " - IR NIR Sensor: OV7251 (V4L2 /dev/video20) -> Dedicated to Howdy"
echo " - IR LED:        TPS68470 WLED2 (Hardware Torch Mode)"
echo ""
echo " To configure Windows Hello face authentication, install Howdy:"
echo "   sudo howdy config"
echo " Set 'device_path = /dev/video20' and 'dark_threshold = 95'."
echo "=========================================================="
