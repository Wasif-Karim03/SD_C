#!/usr/bin/env bash
# install_ch341.sh — build + install the CH340/CH341 USB-serial driver.
#
# Why this exists: this Jetson kernel (5.15.x-tegra) does NOT ship ch341.ko, so
# CH340-based USB-serial adapters (our Arduino Nano clone, USB id 1a86:7523) get
# no /dev/ttyUSB0. This builds the upstream 5.15 ch341 driver against the running
# kernel's headers, installs it, and registers it for udev auto-load on plug.
#
# Re-run this after a kernel update (a new kernel won't have the module).
# Needs: kernel headers (/lib/modules/$(uname -r)/build), gcc, make. Run with
# sudo available — it will prompt for your password at the install step.
set -e
cd "$(dirname "$0")"
KREL="$(uname -r)"
DEST="/lib/modules/$KREL/kernel/drivers/usb/serial"

echo "[1/4] building ch341.ko for kernel $KREL ..."
make clean >/dev/null 2>&1 || true
make

echo "[2/4] installing to $DEST ..."
sudo install -m 644 ch341.ko "$DEST/ch341.ko"

echo "[3/4] depmod (register usb:v1A86p7523 -> ch341 for udev auto-load) ..."
sudo depmod -a

echo "[4/4] loading now ..."
sudo modprobe ch341 || true

echo
echo "Done. Also make sure brltty is removed (it hijacks CH340 ports):"
echo "    sudo apt-get remove -y brltty"
echo
echo "Plug in the Nano and check:  ls -l /dev/ttyUSB*"
