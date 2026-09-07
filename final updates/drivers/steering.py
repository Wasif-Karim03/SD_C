#!/usr/bin/env python3
"""
drivers/steering.py — steering servo via the Arduino Nano (CH340).

Ported from servo/servo_control.py. Path: Jetson --USB serial--> Arduino Nano
(arduino_servo.ino) --D9--> steering servo. Integer-degree ASCII protocol
("90", c, d, s, ?); the Nano echoes "ANGLE nn".

IMPORTANT change from the old code: the Nano is resolved by its STABLE by-id
(1a86 CH340), NOT by "first /dev/ttyUSB*". The RPLIDAR is also a ttyUSB device
(ttyUSB0); picking the first ttyUSB would send steering commands to the LiDAR.

Bench-tested ASYMMETRIC range (linkage binds outside this): left 60 / center 90 /
right 115. steer(-1..+1) maps onto it; clamped here AND in the sketch.

Servo power: the servo needs its OWN 5-6 V BEC with common ground — never the
Jetson 5 V (a stall pulls >1 A and browns things out). Off USB power it may only
twitch/shake under load.
"""
import os
import sys
import glob
import time
import serial

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # single source of truth

BY_ID = config.by_id_path(config.STEERING_BY_ID)
FALLBACK = config.STEERING_FALLBACK
BAUD = config.STEERING_BAUD

CENTER = config.STEER_CENTER
LEFT_LIMIT = config.STEER_LEFT      # norm = -1.0
RIGHT_LIMIT = config.STEER_RIGHT    # norm = +1.0
VESC_ACM = config.VESC_FALLBACK     # never steer this


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def norm_to_angle(norm):
    """Normalized steer (-1 left .. +1 right) -> servo angle (asymmetric halves)."""
    norm = clamp(float(norm), -1.0, 1.0)
    if norm < 0:
        angle = CENTER + norm * (CENTER - LEFT_LIMIT)
    else:
        angle = CENTER + norm * (RIGHT_LIMIT - CENTER)
    return int(round(angle))


def resolve_port():
    """Steering Nano by stable identity; fall back carefully (never the VESC)."""
    if os.path.exists(BY_ID):
        return os.path.realpath(BY_ID)
    # by-id link missing: look for a CH340 (1a86) among by-id names
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if "1a86" in p.lower() or "usb_serial" in p.lower():
            return os.path.realpath(p)
    return FALLBACK if os.path.exists(FALLBACK) else None


class ServoController:
    """Steering interface: steer(-1..+1), set_angle(deg), center(), detach()."""

    def __init__(self, port=None, baud=BAUD, open_now=True):
        self.port = port or resolve_port()
        if not self.port:
            raise RuntimeError("steering Nano not found (by-id 1a86 missing).")
        if os.path.realpath(self.port) == os.path.realpath(VESC_ACM):
            raise RuntimeError("refusing to open the VESC as the steering port.")
        self.baud = baud
        self.ser = None
        self.banner = []
        if open_now:
            self.open()

    def open(self):
        if self.ser is None:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)                 # Nano auto-resets on port open
            self.ser.reset_input_buffer()
            # drain the "READY ..." banner
            deadline = time.time() + 3
            while time.time() < deadline:
                line = self.ser.readline().decode(errors="replace").strip()
                if line:
                    self.banner.append(line)
                if line.startswith("READY"):
                    break
        return self

    def _send(self, cmd, read_reply=True):
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()
        if not read_reply:
            return None
        time.sleep(0.05)
        replies = []
        while True:
            line = self.ser.readline().decode(errors="replace").strip()
            if not line:
                break
            replies.append(line)
        return replies

    def steer(self, norm, read_reply=True):
        return self.set_angle(norm_to_angle(norm), read_reply=read_reply)

    def set_angle(self, deg, read_reply=True):
        deg = int(round(clamp(deg, LEFT_LIMIT, RIGHT_LIMIT)))
        return self._send(str(deg), read_reply=read_reply)

    def center(self, read_reply=True):
        return self._send("c", read_reply=read_reply)

    def detach(self, read_reply=False):
        return self._send("d", read_reply=read_reply)

    def status(self):
        return self._send("?")

    def close(self):
        if self.ser is not None:
            try:
                self.center()
                self.detach()
            except Exception:
                pass
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *a):
        self.close()
