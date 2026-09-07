#!/usr/bin/env bash
# flash.sh — compile + upload arduino_servo.ino to the Nano from the Jetson.
#
# Auto-detects the Nano's /dev/ttyUSB* port and tries the normal Nano
# bootloader first, then falls back to the "Old Bootloader" (atmega328old)
# that most Nano clones need. Run:  ./flash.sh   (or: bash flash.sh)
set -u
export PATH="$HOME/.local/bin:$PATH"

SKETCH_DIR="$(cd "$(dirname "$0")" && pwd)/arduino_servo"

# Find the Nano: prefer ttyUSB*, fall back to a non-VESC ttyACM*.
PORT=""
for p in /dev/ttyUSB*; do [ -e "$p" ] && PORT="$p" && break; done
if [ -z "$PORT" ]; then
  for p in /dev/ttyACM*; do
    [ -e "$p" ] && [ "$p" != "/dev/ttyACM0" ] && PORT="$p" && break
  done
fi
if [ -z "$PORT" ]; then
  echo "No Nano port found. Plug it in and check: ls /dev/ttyUSB* /dev/ttyACM*"
  exit 1
fi
echo "Using port: $PORT"

echo "Compiling $SKETCH_DIR ..."
if ! arduino-cli compile --fqbn arduino:avr:nano "$SKETCH_DIR"; then
  echo "Compile failed."; exit 1
fi

echo "Uploading (normal bootloader) ..."
if arduino-cli upload -p "$PORT" --fqbn arduino:avr:nano "$SKETCH_DIR"; then
  echo "Upload OK."; exit 0
fi

echo "Normal bootloader failed — retrying with Old Bootloader (atmega328old) ..."
if arduino-cli upload -p "$PORT" --fqbn arduino:avr:nano:cpu=atmega328old "$SKETCH_DIR"; then
  echo "Upload OK (old bootloader)."; exit 0
fi

echo "Both upload attempts failed. Check the cable/port and that nothing else"
echo "(Serial Monitor, servo_control.py) is holding $PORT open."
exit 1
