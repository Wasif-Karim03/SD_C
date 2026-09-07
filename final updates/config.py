#!/usr/bin/env python3
"""
config.py — THE single source of truth for every piece of hardware on the car.

Nothing else in `final updates/` should hardcode a /dev path, a baud rate, an I2C
address, or a mechanical limit. Every driver imports what it needs from here. If
the wiring/identity ever changes, THIS is the one file to edit.

Why by-id / by-path instead of /dev/ttyUSB0 or /dev/video0?
  Those numbers are assigned in enumeration order and SHIFT when a device is
  added/removed/replugged (adding the LiDAR already pushed steering ttyUSB0->1).
  Stable identities under /dev/serial/by-id and /dev/v4l/by-path never move, so we
  resolve by those and only fall back to a raw path if the stable link is missing.

Verified working 2026-08-08: every device below resolved and passed its test.
"""
import os

BY_ID_DIR = "/dev/serial/by-id"


def _resolve(by_id_name, fallback):
    """Return the real /dev path for a by-id name, else `fallback`."""
    link = os.path.join(BY_ID_DIR, by_id_name)
    return os.path.realpath(link) if os.path.exists(link) else fallback


def by_id_path(by_id_name):
    """Full /dev/serial/by-id/... path for a name (whether or not it exists now)."""
    return os.path.join(BY_ID_DIR, by_id_name)


# --------------------------------------------------------------------------- #
# USB / serial devices
# --------------------------------------------------------------------------- #
# VESC — Flipsky Mini FSESC 6.7 Pro (drive motor). Only enumerates with the MOTOR
# BATTERY on (logic is battery-fed, not USB).
VESC_BY_ID = "usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
VESC_FALLBACK = "/dev/ttyACM0"
VESC_BAUD = 115200

# Steering — Arduino Nano clone (CH340 / 1a86), servo signal on D9.
STEERING_BY_ID = "usb-1a86_USB_Serial-if00-port0"
STEERING_FALLBACK = "/dev/ttyUSB1"
STEERING_BAUD = 115200

# RPLIDAR C1 — Silicon Labs CP2102N bridge.
LIDAR_BY_ID = ("usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
               "cad4bd81365aee11899081dc8ffcc75d-if00-port0")
LIDAR_FALLBACK = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

# GPS — Radiolink SE100 (u-blox M8N), 40-pin header UART (pins 8/10 crossed).
# Not a USB device -> raw UART path, no by-id.
GPS_PORT = "/dev/ttyTHS1"
GPS_BAUD = 38400


def vesc_port():
    return _resolve(VESC_BY_ID, VESC_FALLBACK)


def steering_port():
    return _resolve(STEERING_BY_ID, STEERING_FALLBACK)


def lidar_port():
    return _resolve(LIDAR_BY_ID, LIDAR_FALLBACK)


# --------------------------------------------------------------------------- #
# I2C devices (compass + OLED share bus 7 on header pins 3/5)
# --------------------------------------------------------------------------- #
I2C_BUS = 7
COMPASS_ADDR = 0x0E    # IST8310, WHO_AM_I(0x00)=0x10
OLED_ADDR = 0x3C       # SSD1306, VCC on 3.3V

# --------------------------------------------------------------------------- #
# Cameras — two identical icSpring USB cams, pinned by PHYSICAL USB PORT PATH
# (identical models can't be told apart by by-id; port path is stable).
# --------------------------------------------------------------------------- #
CAM_FRONT_BYPATH = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0"
CAM_REAR_BYPATH = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0"
CAM_FRONT_FALLBACK = "/dev/video0"
CAM_REAR_FALLBACK = "/dev/video2"
CAM_REAR_ROTATE = 180        # rear camera is mounted upside down
CAM_FRONT_ROTATE = 0
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_PIXEL_FORMAT = "YUYV"    # only format the sensor advertises (no MJPG)
CAM_FPS_CEILING = 22
# Manual exposure (units of 100us; 312 ~= 31 ms). Lower (~156) to cut motion blur.
CAM_MANUAL_EXPOSURE = True
CAM_EXPOSURE_ABS = 312

# --------------------------------------------------------------------------- #
# Steering geometry (bench-tested, ASYMMETRIC — linkage swings further left)
# Keep in sync with the Nano sketch STEER_MIN/STEER_MAX.
# --------------------------------------------------------------------------- #
STEER_CENTER = 90
STEER_LEFT = 60      # norm = -1.0 (full left)
STEER_RIGHT = 115    # norm = +1.0 (full right)

# --------------------------------------------------------------------------- #
# Vehicle model / calibration
# --------------------------------------------------------------------------- #
METERS_PER_TACH = 0.003424   # CALIBRATED 2026-07-23 (6.8 m -> Δtach 1986)
MAX_DUTY = 0.20              # hard throttle ceiling for autonomy
DEFAULT_DUTY = 0.05         # gentle bench-tested start
WHEELBASE_M = 0.25          # TODO measure on the real chassis
MAX_STEER_ANGLE_RAD = 0.45  # TODO measure on the real chassis

# --------------------------------------------------------------------------- #
# Perception — free-space thresholds (metres)
# Tuned to THIS camera + Depth-Anything V2 indoor: absolute metric depth runs
# COMPRESSED/drifty indoors, so an open corridor reads ~2.6 m and a close obstacle
# ~0.6 m (measured 2026-08-08). Block well below the open reading. These are
# environment-tuned, not physical truth — re-tune on the real driving floor.
# --------------------------------------------------------------------------- #
FREESPACE_STOP_M = 1.2      # declare BLOCKED when the center view caps below this
FREESPACE_CLEAR_M = 1.6     # ...and only clear once it opens past this (hysteresis)
FREESPACE_SLOW_REACH = 3.5  # full speed only when it sees this far; scale down below

# Sensorless BLDC won't reliably start from a standstill below ~5-6% duty (it just
# cogs). When we intend to move, floor the command here so it actually rolls.
MIN_MOVE_DUTY = 0.06

# --------------------------------------------------------------------------- #
# LiDAR-based navigation (metric, 360°). LiDAR is the geometry/path sensor.
# LIDAR_FORWARD_DEG = the raw scan angle that points to the car's FRONT.
#
# CALIBRATE WITH apps/calibrate_signs.py — NOT with "the nearest return". The
# nearest return is whatever permanent thing sits closest to the LiDAR (chassis, a
# stand leg, a person watching), which is how this ended up at 340 when it is
# really ~354. calibrate_signs.py takes a baseline first and then looks for the
# bearings whose range DROPPED when you place a box, so permanent clutter cancels.
# Cross-checked 2026-09-07: nose box -> 354.6, and the left/right box pair (86.5 /
# 269.5 raw, 183 deg apart) independently implies ~358. Agreement within a few
# degrees; lidar_nav_report.txt's differential cluster said 353.
# --------------------------------------------------------------------------- #
LIDAR_FORWARD_DEG = 354.6    # MEASURED by apps/calibrate_signs.py

# --------------------------------------------------------------------------- #
# STEER_SIGN — ONE knob for every "which way do I turn?" decision on the car.
#
# Both the pure-pursuit follower and the LiDAR avoider ask the same underlying
# physical question: as the raw LiDAR scan angle INCREASES, does the bearing sweep
# clockwise (toward the car's right) or counter-clockwise (toward its left)? The
# SLAM map is built straight from those raw angles, so the map frame inherits the
# same handedness — which means the follower's heading error and the avoider's
# left/right comparison flip together, not independently. Keeping two separate
# sign constants guaranteed one of them was wrong (they were set to opposite
# values), so there is now exactly one.
#
#   +1.0  scan angle increases CLOCKWISE  (rel > 0 is the car's RIGHT)
#   -1.0  scan angle increases COUNTER-CLOCKWISE (rel > 0 is the car's LEFT)
#
# SET BY apps/calibrate_signs.py (step 3). Do not guess it.
# --------------------------------------------------------------------------- #
STEER_SIGN = 1.0    # MEASURED by apps/calibrate_signs.py
STEER_SIGN_VERIFIED = True   # measured, not guessed

LIDAR_STEER_SIGN = STEER_SIGN   # kept as an alias; edit STEER_SIGN, not this
LIDAR_FRONT_ARC_DEG = 60.0   # +/- this from forward = the "ahead" cone (blocking)
LIDAR_STEER_ARC_DEG = 120.0  # arc scanned to choose the most-open direction
LIDAR_STOP_M = 0.5           # BLOCKED if nearest obstacle in the front cone < this
LIDAR_CLEAR_M = 0.8          # ...and only clear once it opens past this (hysteresis)
LIDAR_MIN_M = 0.15           # hard floor: ignore returns closer than this always.
                             # The car's own body is masked PER-BEARING by the self
                             # profile below (chassis returns span a band, so one
                             # global number can't catch them).

# Per-bearing self-return mask (the car's own body/mast). Generated by
# apps/calibrate_lidar_self.py -> lidar_self_profile.json: a list of 360 mask radii
# in metres (0 = no mask). ThreadedLidar drops any return closer than the mask at
# that bearing, so the chassis never reads as an obstacle (esp. directly behind).
LIDAR_SELF_PROFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "lidar_self_profile.json")


def load_self_mask():
    import json
    try:
        with open(LIDAR_SELF_PROFILE) as fh:
            m = json.load(fh)
        return m if isinstance(m, list) and len(m) == 360 else None
    except Exception:
        return None

# --------------------------------------------------------------------------- #
# Object detection (YOLO11n, TensorRT) — reuse the engine built in the old work.
# --------------------------------------------------------------------------- #
_PROJ = os.path.expanduser("~/Documents/Self Driving Car")
YOLO_ENGINE = os.path.join(_PROJ, "Camera Control", "yolo11n.engine")
YOLO_PT = os.path.join(_PROJ, "Camera Control", "yolo11n.pt")
YOLO_CONF = 0.35             # min confidence to report a detection
YOLO_IMGSZ = 640            # must match the engine's build size
# classes that get extra caution (COCO ids): person, bicycle, car, motorcycle,
# bus, truck, cat, dog
VRU_CLASSES = {0, 1, 2, 3, 5, 7, 15, 16}


if __name__ == "__main__":
    # Quick dump of what resolves right now.
    print("VESC     :", vesc_port(), "(present)" if os.path.exists(vesc_port()) else "(MISSING)")
    print("Steering :", steering_port(), "(present)" if os.path.exists(steering_port()) else "(MISSING)")
    print("LiDAR    :", lidar_port(), "(present)" if os.path.exists(lidar_port()) else "(MISSING)")
    print("GPS      :", GPS_PORT, "(present)" if os.path.exists(GPS_PORT) else "(MISSING)")
    print("I2C bus  :", f"/dev/i2c-{I2C_BUS}",
          "(present)" if os.path.exists(f"/dev/i2c-{I2C_BUS}") else "(MISSING)")
    print("Cam front:", CAM_FRONT_BYPATH, "(present)" if os.path.exists(CAM_FRONT_BYPATH) else "(MISSING)")
    print("Cam rear :", CAM_REAR_BYPATH, "(present)" if os.path.exists(CAM_REAR_BYPATH) else "(MISSING)")
