#!/usr/bin/env python3
"""
devices.py — resolve the VESC and Arduino to STABLE serial paths by identity.

/dev/ttyACM0 and /dev/ttyACM1 can swap on replug/reboot. Sending a throttle ramp
to the steering Arduino (or vice-versa) would be dangerous, so we always look the
devices up by their USB identity under /dev/serial/by-id instead of by number.
"""

import glob


def _find(substr, fallback):
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if substr.lower() in p.lower():
            return p
    return fallback


# VESC = STM32 virtual COM port; Arduino Uno reports "Arduino".
VESC_PORT = _find("STMicroelectronics", "/dev/ttyACM0")
ARDUINO_PORT = _find("Arduino", "/dev/ttyACM1")


if __name__ == "__main__":
    print("VESC   :", VESC_PORT)
    print("Arduino:", ARDUINO_PORT)
