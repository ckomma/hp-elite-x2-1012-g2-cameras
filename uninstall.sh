#!/bin/bash
# uninstall.sh - Uninstaller for HP Elite x2 1012 G2 Camera & IR System
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Error: This script must be run as root (e.g. sudo ./uninstall.sh)" >&2
  exit 1
fi

PACKAGE_NAME="hp-camera-unified"
PACKAGE_VERSION="1.0.0"

echo "=== 1. Stopping & Disabling Services ==="
systemctl stop hp-ir-feeder.service 2>/dev/null || true
systemctl disable hp-ir-feeder.service 2>/dev/null || true
systemctl stop hp-camera-unified.service 2>/dev/null || true
systemctl disable hp-camera-unified.service 2>/dev/null || true

echo "=== 2. Removing Systemd Units & Scripts ==="
rm -f /etc/systemd/system/hp-camera-unified.service
rm -f /etc/systemd/system/hp-ir-feeder.service
rm -f /usr/local/bin/hp-ir-feeder
rm -f /usr/local/libexec/hp-camera-unified
rm -f /etc/modprobe.d/hp-camera-unified.conf
rm -f /etc/wireplumber/wireplumber.conf.d/50-disable-hp-ir-camera.conf
systemctl daemon-reload

echo "=== 3. Removing DKMS Module ==="
if dkms status | grep -q "${PACKAGE_NAME}/${PACKAGE_VERSION}"; then
  dkms remove "${PACKAGE_NAME}/${PACKAGE_VERSION}" --all || true
fi
rm -rf "/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}"

echo "Uninstallation completed cleanly."
