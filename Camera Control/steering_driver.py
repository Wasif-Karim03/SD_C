#!/usr/bin/env python3
"""
steering_driver.py — Jetson-side steering control via the Arduino.

Sends a steering command (-1.0 left .. +1.0 right) to the Arduino running
steering_arduino.ino, which generates the servo pulse. The Arduino re-centers
on its own if commands stop (failsafe), so the consumer must send continuously.
"""

import time

import serial

from devices import ARDUINO_PORT


class ArduinoSteering:
    def __init__(self, port=ARDUINO_PORT, baud=115200, reset_wait=2.0):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(reset_wait)   # Uno auto-resets when the port opens
        self.center()

    def steer(self, value):
        """value in [-1.0, +1.0]: -1 = full left, 0 = center, +1 = full right."""
        value = max(-1.0, min(1.0, float(value)))
        self.ser.write(f"{value:.3f}\n".encode())

    def center(self):
        self.steer(0.0)

    def close(self):
        try:
            self.center()
            time.sleep(0.05)
        finally:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


if __name__ == "__main__":
    # Standalone sweep test.
    s = ArduinoSteering()
    for val, lbl in [(0, "center"), (-1, "left"), (0, "center"), (1, "right"), (0, "center")]:
        print("steer", lbl)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.2:
            s.steer(val); time.sleep(0.05)
    s.close()
