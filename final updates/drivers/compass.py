#!/usr/bin/env python3
"""
drivers/compass.py — Radiolink SE100 IST8310 magnetometer (heading).

Ported from GPS/compass_read.py. IST8310 on /dev/i2c-7 @ 0x0e (40-pin header pins
3/SDA, 5/SCL), sharing the bus with the SSD1306 OLED (0x3c).

  open()   -> confirm WHO_AM_I, configure averaging, load calibration if present
  read()   -> dict(x, y, z uT, heading_deg, calibrated)

WARNING (from the hardware log): motor/phase currents corrupt this compass. Read it
away from the motor, and calibrate (360deg spin) in the final mounted position.
Uncalibrated readings are RAW — direction is only roughly right.
"""
import os
import sys
import json
import time
import math
from smbus2 import SMBus

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # single source of truth

BUS = config.I2C_BUS
ADDR = config.COMPASS_ADDR

REG_WHOAMI = 0x00    # -> 0x10
REG_STAT1 = 0x02     # bit0 = data ready
REG_DATA = 0x03      # X_L,X_H,Y_L,Y_H,Z_L,Z_H (little-endian signed)
REG_CNTL1 = 0x0A     # 0x01 = single measurement
REG_AVG = 0x41       # averaging
REG_PDCNTL = 0x42    # pulse duration
SENS = 0.3           # ~0.3 uT per LSB

# where a saved calibration might live (first that exists wins)
CAL_CANDIDATES = [
    os.path.join(os.path.dirname(__file__), "compass_cal.json"),
    os.path.expanduser("~/Documents/Self Driving Car/GPS/compass_cal.json"),
]


def _s16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def _load_cal():
    for path in CAL_CANDIDATES:
        if os.path.exists(path):
            try:
                with open(path) as fh:
                    c = json.load(fh)
                return {"offset": c.get("offset", [0, 0, 0]),
                        "scale": c.get("scale", [1, 1, 1]), "path": path}
            except Exception:
                pass
    return {"offset": [0, 0, 0], "scale": [1, 1, 1], "path": None}


class Compass:
    def __init__(self, bus=BUS, addr=ADDR):
        self.bus_num = bus
        self.addr = addr
        self.bus = None
        self.cal = _load_cal()
        self.whoami = None

    def open(self):
        self.bus = SMBus(self.bus_num)
        self.whoami = self.bus.read_byte_data(self.addr, REG_WHOAMI)
        # 16x averaging, normal pulse duration
        self.bus.write_byte_data(self.addr, REG_AVG, 0x24)
        self.bus.write_byte_data(self.addr, REG_PDCNTL, 0xC0)
        return self

    @property
    def is_calibrated(self):
        return self.cal["path"] is not None

    def read(self):
        self.bus.write_byte_data(self.addr, REG_CNTL1, 0x01)   # trigger
        for _ in range(20):                                    # wait data-ready
            if self.bus.read_byte_data(self.addr, REG_STAT1) & 0x01:
                break
            time.sleep(0.002)
        d = self.bus.read_i2c_block_data(self.addr, REG_DATA, 6)
        x = _s16(d[0], d[1]) * SENS
        y = _s16(d[2], d[3]) * SENS
        z = _s16(d[4], d[5]) * SENS
        ox, oy, oz = self.cal["offset"]
        sx, sy, sz = self.cal["scale"]
        x, y, z = (x - ox) * sx, (y - oy) * sy, (z - oz) * sz
        heading = math.degrees(math.atan2(y, x))
        if heading < 0:
            heading += 360.0
        return {"x": x, "y": y, "z": z, "heading_deg": heading,
                "calibrated": self.is_calibrated}

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *a):
        self.close()
