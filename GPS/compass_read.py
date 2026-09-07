#!/usr/bin/env python3
"""Heading readout for the Radiolink SE100's IST8310 magnetometer.

The SE100 carries an IST8310 compass on the same I2C bus as the GPS UART.
On this Jetson Orin Nano it shows up at 0x0e on /dev/i2c-7 (40-pin header
pins 3/SDA, 5/SCL) — the same two lines the SSD1306 OLED (0x3c) shares.

Usage: python3 compass_read.py [bus] [addr]
Defaults: bus 7, addr 0x0e

Verify it is present first:  i2cdetect -y -r 7   (expect 0x0e and 0x3c)
Ctrl-C to stop.
"""
import sys
import time
import math
from smbus2 import SMBus

import compass_cal

BUS = int(sys.argv[1]) if len(sys.argv) > 1 else 7
ADDR = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x0E

# IST8310 register map
REG_WHOAMI = 0x00   # -> 0x10
REG_STAT1 = 0x02    # bit0 = data ready
REG_DATA = 0x03     # X_L, X_H, Y_L, Y_H, Z_L, Z_H (little-endian, signed)
REG_CNTL1 = 0x0A    # 0x01 = single measurement
REG_CNTL2 = 0x0B
REG_AVG = 0x41      # averaging
REG_PDCNTL = 0x42   # pulse duration

SENS = 0.3 / 1.0    # ~0.3 microtesla per LSB


def s16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def main():
    bus = SMBus(BUS)

    who = bus.read_byte_data(ADDR, REG_WHOAMI)
    if who != 0x10:
        print(f"WARNING: WHO_AM_I=0x{who:02x} (expected 0x10 for IST8310)")
    else:
        print(f"IST8310 found at 0x{ADDR:02x} on i2c-{BUS} (WHO_AM_I=0x10)")

    # Recommended setup: 16x averaging, normal pulse duration
    bus.write_byte_data(ADDR, REG_AVG, 0x24)
    bus.write_byte_data(ADDR, REG_PDCNTL, 0xC0)

    cal = compass_cal.load()
    if compass_cal.is_identity(cal):
        print("NOTE: no compass_cal.json — readings are RAW/uncalibrated. "
              "Run compass_calibrate.py for trustworthy bearings.")
    else:
        off = cal["offset"]
        print("calibration loaded: offset "
              f"X={off[0]:+.1f} Y={off[1]:+.1f} Z={off[2]:+.1f} uT")

    print("Reading heading... (Ctrl-C to stop)")
    print("NOTE: keep away from motors/magnets. "
          "Rotate the car flat to sanity-check N/E/S/W.\n")
    try:
        while True:
            # trigger one measurement
            bus.write_byte_data(ADDR, REG_CNTL1, 0x01)
            # wait for data-ready (datasheet ~6ms)
            for _ in range(20):
                if bus.read_byte_data(ADDR, REG_STAT1) & 0x01:
                    break
                time.sleep(0.002)

            d = bus.read_i2c_block_data(ADDR, REG_DATA, 6)
            x = s16(d[0], d[1]) * SENS
            y = s16(d[2], d[3]) * SENS
            z = s16(d[4], d[5]) * SENS
            x, y, z = compass_cal.apply(x, y, z, cal)

            # heading in the X-Y plane (board flat). 0=+X axis.
            heading = math.degrees(math.atan2(y, x))
            if heading < 0:
                heading += 360.0

            print(f"X={x:8.1f}  Y={y:8.1f}  Z={z:8.1f} uT   "
                  f"heading={heading:6.1f} deg", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
