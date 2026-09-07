#!/usr/bin/env python3
"""
compass_calibrate.py — hard-iron (+ simple soft-iron) calibration for the
Radiolink SE100's IST8310 magnetometer.

Run this ONCE, with the compass in its FINAL mounted position (the box/chassis
metal is part of the offset, so calibrating off the car gives the wrong answer).

Slowly rotate the car *flat* through several full 360 turns while it samples.
It tracks the per-axis min/max and computes:

    offset[i] = (max[i] + min[i]) / 2        # hard-iron: recenter the field
    scale[i]  = mean_radius / radius[i]      # soft-iron: re-round the ellipse

then writes them to compass_cal.json next to this script. compass_read.py and
gps_web.py load that file and subtract the offset (and apply scale) before
heading = atan2(Y, X), so bearings become trustworthy.

    python3 compass_calibrate.py [bus] [addr]      (defaults 7, 0x0e)

Heading uses X/Y, so a FLAT spin is what matters. Add some pitch/roll near the
end if you also want Z usable (e.g. for future tilt compensation).
Press Ctrl-C when the ranges stop growing to finish and save.

NOTE: redo this after the motor is wired. Motor phase current adds a
throttle-dependent field that this static calibration cannot remove; if the
heading swings when driving, the compass must move to a standoff away from the
VESC/motor/battery wiring.
"""
import sys
import time
import math
import json
import datetime
from smbus2 import SMBus

import compass_cal

BUS = int(sys.argv[1]) if len(sys.argv) > 1 else 7
ADDR = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x0E

MAG_SENS = 0.3  # microtesla per LSB
# An axis whose full swing is below this (uT) was never properly rotated
# through the field, so its scale/offset is unreliable -> leave it identity.
MIN_RANGE_UT = 8.0


def s16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def read_xyz(bus):
    """Trigger one IST8310 measurement and return (x, y, z) in microtesla."""
    bus.write_byte_data(ADDR, 0x0A, 0x01)        # single measurement
    for _ in range(20):
        if bus.read_byte_data(ADDR, 0x02) & 0x01:  # data-ready
            break
        time.sleep(0.002)
    d = bus.read_i2c_block_data(ADDR, 0x03, 6)
    return (s16(d[0], d[1]) * MAG_SENS,
            s16(d[2], d[3]) * MAG_SENS,
            s16(d[4], d[5]) * MAG_SENS)


def compute(lo, hi):
    """Turn per-axis min/max into a {'offset','scale'} calibration dict."""
    offset = [(hi[i] + lo[i]) / 2.0 for i in range(3)]
    radius = [(hi[i] - lo[i]) / 2.0 for i in range(3)]
    # Mean radius over only the well-sampled axes (flat spin barely moves Z).
    good = [radius[i] for i in range(3) if (hi[i] - lo[i]) >= MIN_RANGE_UT]
    mean_r = sum(good) / len(good) if good else 0.0
    scale = []
    for i in range(3):
        if (hi[i] - lo[i]) >= MIN_RANGE_UT and radius[i] > 0 and mean_r > 0:
            scale.append(mean_r / radius[i])
        else:
            scale.append(1.0)          # under-sampled axis -> no scaling
    return {"offset": offset, "scale": scale, "radius": radius}


def main():
    bus = SMBus(BUS)
    who = bus.read_byte_data(ADDR, 0x00)
    if who != 0x10:
        print(f"WARNING: WHO_AM_I=0x{who:02x} (expected 0x10 for IST8310)")
    else:
        print(f"IST8310 found at 0x{ADDR:02x} on i2c-{BUS}.")
    bus.write_byte_data(ADDR, 0x41, 0x24)  # 16x averaging
    bus.write_byte_data(ADDR, 0x42, 0xC0)  # pulse duration

    print("\n>>> Rotate the car SLOWLY and FLAT through several full 360 turns.")
    print(">>> Watch the X/Y ranges grow; tilt a bit at the end for Z.")
    print(">>> Press Ctrl-C when the ranges stop changing to save.\n")

    # Prime one reading to seed the min/max.
    x, y, z = read_xyz(bus)
    lo = [x, y, z]
    hi = [x, y, z]
    n = 1
    try:
        while True:
            x, y, z = read_xyz(bus)
            for i, v in enumerate((x, y, z)):
                if v < lo[i]:
                    lo[i] = v
                if v > hi[i]:
                    hi[i] = v
            n += 1
            rng = [hi[i] - lo[i] for i in range(3)]
            print(f"\rsamples {n:5d}  rangeX {rng[0]:6.1f}  "
                  f"rangeY {rng[1]:6.1f}  rangeZ {rng[2]:6.1f} uT   ",
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nfinishing...")
    finally:
        bus.close()

    cal = compute(lo, hi)
    out = {
        "offset": [round(v, 2) for v in cal["offset"]],
        "scale": [round(v, 4) for v in cal["scale"]],
        "samples": n,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "note": "hard-iron + simple soft-iron, IST8310, static (motor off)",
    }

    rng = [hi[i] - lo[i] for i in range(3)]
    print(f"\nhard-iron offset (uT): X={out['offset'][0]:+.1f}  "
          f"Y={out['offset'][1]:+.1f}  Z={out['offset'][2]:+.1f}")
    print(f"soft-iron scale     : X={out['scale'][0]:.3f}  "
          f"Y={out['scale'][1]:.3f}  Z={out['scale'][2]:.3f}")
    for i, ax in enumerate("XYZ"):
        if rng[i] < MIN_RANGE_UT:
            print(f"  ! {ax} range only {rng[i]:.1f} uT (<{MIN_RANGE_UT}) — "
                  f"not enough rotation on this axis; left uncalibrated.")

    if rng[0] < MIN_RANGE_UT or rng[1] < MIN_RANGE_UT:
        print("\nX or Y was under-rotated — heading needs both. "
              "Re-run and spin through full flat circles before saving.")

    with open(compass_cal.CAL_PATH, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nsaved -> {compass_cal.CAL_PATH}")
    print("compass_read.py and gps_web.py will pick it up on next start.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
