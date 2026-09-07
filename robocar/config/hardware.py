#!/usr/bin/env python3
"""
hardware.py — the SINGLE source of truth for every piece of hardware on the car.

Nothing else in `robocar/` should hardcode a /dev path, a baud rate, or a
mechanical limit. Everything imports it from here. If the wiring ever changes,
this is the ONE file to edit.

Why by-id and not /dev/ttyUSB0?
------------------------------
/dev/ttyUSB0, ttyUSB1, ttyACM0 are assigned in *enumeration order* and SHIFT
whenever a device is added, removed, or replugged. Adding the RPLIDAR, for
example, pushed the steering Nano from ttyUSB0 to ttyUSB1 — so any script that
grabbed "the first ttyUSB*" would suddenly be talking to the LiDAR instead of
the steering board. That is dangerous.

Every USB device instead has a STABLE identity under /dev/serial/by-id/ that
never moves. We resolve by that identity and only fall back to a raw path if the
by-id link is missing.

Verified live on 2026-08-08 (all three by-id links present and pointing where
this file says):
    VESC     by-id -> /dev/ttyACM0
    LiDAR    by-id -> /dev/ttyUSB0
    Steering by-id -> /dev/ttyUSB1
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass, field


# --------------------------------------------------------------------------- #
# Port resolution helpers
# --------------------------------------------------------------------------- #
BY_ID_DIR = "/dev/serial/by-id"


def resolve(by_id: str, fallback: str | None = None) -> str | None:
    """Return the real /dev path for a by-id name, or `fallback` if it's absent.

    `by_id` is the filename under /dev/serial/by-id/ (not the full path).
    Returns None if it's missing and no fallback is given — callers can then
    decide whether that device is required for what they're doing.
    """
    link = os.path.join(BY_ID_DIR, by_id)
    if os.path.exists(link):
        return os.path.realpath(link)
    return fallback


def present(by_id: str) -> bool:
    """True if a device with this by-id identity is currently plugged in."""
    return os.path.exists(os.path.join(BY_ID_DIR, by_id))


# --------------------------------------------------------------------------- #
# Device descriptor
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SerialDevice:
    """A USB/UART device resolved by stable identity."""
    name: str          # human label
    by_id: str         # filename under /dev/serial/by-id/ ("" if not a USB-serial device)
    fallback: str      # raw /dev path to use if the by-id link is missing
    baud: int
    note: str = ""

    @property
    def port(self) -> str | None:
        """The resolved /dev path right now (by-id preferred), or None if absent."""
        if self.by_id:
            return resolve(self.by_id, self.fallback if os.path.exists(self.fallback) else None)
        return self.fallback if os.path.exists(self.fallback) else None

    @property
    def connected(self) -> bool:
        return self.port is not None


# --------------------------------------------------------------------------- #
# THE DEVICES  (edit here and nowhere else)
# --------------------------------------------------------------------------- #

# Drive motor controller — Flipsky Mini FSESC 6.7 Pro (VESC HW 6.x, fw 5.2).
# Native VESC packet protocol over USB. NOTE: the VESC only enumerates while its
# MOTOR BATTERY is powered (logic is battery-fed, not USB) — no battery => no port.
VESC = SerialDevice(
    name="VESC (drive motor)",
    by_id="usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00",
    fallback="/dev/ttyACM0",
    baud=115200,
    note="Throttle + telemetry. Needs motor battery on to appear.",
)

# Steering — Arduino Nano clone (CH340 / 1a86) running arduino_servo.ino, servo on D9.
# Integer-angle serial protocol ("90", c, d, s, ?). Servo needs its OWN 5-6V supply
# (BEC), common ground with the Nano — never powered from the Jetson.
STEERING = SerialDevice(
    name="Steering Nano (CH340)",
    by_id="usb-1a86_USB_Serial-if00-port0",
    fallback="/dev/ttyUSB1",
    baud=115200,
    note="Arduino Nano, servo signal on D9. Integer-degree protocol.",
)

# 2D LiDAR — RPLIDAR C1 (Silicon Labs CP2102N bridge). Model 0x41, fw 1.01.
LIDAR = SerialDevice(
    name="RPLIDAR C1",
    by_id="usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_cad4bd81365aee11899081dc8ffcc75d-if00-port0",
    fallback="/dev/ttyUSB0",
    baud=460800,
    note="360deg 2D scan. Set DTR low on open so the motor isn't held in reset.",
)

# GPS — Radiolink SE100 (u-blox M8N), NMEA on the Jetson 40-pin header UART.
# Pins 8/10, TX/RX crossed. Not a USB device -> no by-id, use the raw UART path.
GPS = SerialDevice(
    name="GPS SE100 (u-blox M8N)",
    by_id="",
    fallback="/dev/ttyTHS1",
    baud=38400,   # SE100 ships at 38400, NOT the u-blox 9600 default
    note="40-pin header UART, pins 8/10 crossed. Needs sky view for a fix.",
)


# --------------------------------------------------------------------------- #
# I2C devices (compass + OLED share one bus)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class I2CDevice:
    name: str
    bus: int
    addr: int
    whoami_reg: int | None = None
    whoami_val: int | None = None
    note: str = ""

    @property
    def dev(self) -> str:
        return f"/dev/i2c-{self.bus}"


# IST8310 magnetometer (compass) — bus 7 @ 0x0e, WHO_AM_I(0x00)==0x10.
# Shares SDA/SCL (header pins 3/5) with the OLED — only one I2C pin pair exists.
# WARNING: motor/phase currents corrupt this compass — mast-mount away from VESC.
COMPASS = I2CDevice(
    name="IST8310 compass",
    bus=7, addr=0x0E, whoami_reg=0x00, whoami_val=0x10,
    note="Shares i2c-7 with OLED 0x3c. Uncalibrated + motor-sensitive.",
)

# SSD1306 OLED stats display — bus 7 @ 0x3c.
OLED = I2CDevice(
    name="SSD1306 OLED",
    bus=7, addr=0x3C,
    note="Shares i2c-7 with the compass.",
)


# --------------------------------------------------------------------------- #
# Camera
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Camera:
    name: str = "icSpring USB camera"
    # Two identical icSpring cameras enumerate on video0..video3; the driver
    # picks the first working USB capture node at runtime. We keep a hint here
    # but the driver layer will confirm by opening it.
    device_hint: str = "/dev/video0"
    width: int = 640
    height: int = 480
    pixel_format: str = "YUYV"      # only format the sensor advertises (no MJPG)
    fps_ceiling: int = 22           # practical cap; camera is the FPS bottleneck
    # Manual exposure pinned so auto-exposure can't drop FPS in low light.
    # Units of exposure_time_absolute are 100us, so 312 ~= 31 ms. Lower (e.g.
    # 156) to cut motion blur once the car is moving.
    manual_exposure: bool = True
    exposure_time_absolute: int = 312


CAMERA = Camera()


# --------------------------------------------------------------------------- #
# Steering geometry  (bench-tested, ASYMMETRIC — the linkage swings further left)
# Keep in sync with servo/arduino_servo.ino (STEER_MIN / STEER_MAX) and
# servo/servo_control.py (LEFT_LIMIT / RIGHT_LIMIT).
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SteeringGeometry:
    center_deg: int = 90
    left_deg: int = 60       # norm = -1.0 (full left)
    right_deg: int = 115     # norm = +1.0 (full right)


STEER_GEOM = SteeringGeometry()


# --------------------------------------------------------------------------- #
# Vehicle model / calibration  (used by odometry + any Ackermann math)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class VehicleModel:
    # CALIBRATED 2026-07-23: pushed 6.8 m -> Δtach 1986 (~292 counts/m).
    meters_per_tach: float = 0.003424
    # TODO measure these two on the real chassis — they set the turning radius.
    wheelbase_m: float = 0.25          # provisional
    max_steer_angle_rad: float = 0.45  # provisional
    # Throttle safety caps (duty fraction, 0..1). Start gentle.
    max_duty: float = 0.10             # hard ceiling for camera autonomy
    default_duty: float = 0.05         # bench-tested comfortable start


VEHICLE = VehicleModel()


# --------------------------------------------------------------------------- #
# Self-test:  python3 -m robocar.config.hardware   (or run this file directly)
# --------------------------------------------------------------------------- #
def status_report() -> str:
    lines = ["=== robocar hardware status ==="]
    for d in (VESC, STEERING, LIDAR, GPS):
        state = d.port if d.connected else "MISSING"
        lines.append(f"[serial] {d.name:24s} {state}  @{d.baud}"
                     + ("" if d.connected else "  <-- not plugged in / powered?"))
    for d in (COMPASS, OLED):
        exists = os.path.exists(d.dev)
        lines.append(f"[i2c]    {d.name:24s} {d.dev} @ 0x{d.addr:02x}"
                     + ("" if exists else "  <-- bus missing"))
    cams = sorted(glob.glob("/dev/video*"))
    lines.append(f"[camera] {CAMERA.name:24s} nodes: {', '.join(cams) or 'none'}")
    return "\n".join(lines)


if __name__ == "__main__":
    print(status_report())
