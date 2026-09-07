#!/usr/bin/env python3
"""
apps/test_compass.py — prove the IST8310 compass works.

Confirms WHO_AM_I (0x10), then streams heading for a window. To sanity-check it,
SLOWLY ROTATE THE CAR FLAT through a full circle while it runs — the heading should
sweep smoothly through 0->360 and the X/Y values should trace a circle.

Reads on /dev/i2c-7 @ 0x0e. Keep away from the motor/large metal while testing;
uncalibrated readings are RAW (direction roughly right, not a true bearing).

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_compass.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from drivers.compass import Compass   # noqa: E402

REPORT = os.path.join(HERE, "compass_report.txt")
WINDOW = 12.0

lines = []


def log(m=""):
    print(m)
    lines.append(m)


def main():
    log("=" * 64)
    log(f"IST8310 compass test @ {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log("=" * 64)
    c = Compass()
    dev = f"/dev/i2c-{c.bus_num}"
    if not os.path.exists(dev):
        log(f"  !! {dev} missing — I2C bus not present.")
        _save()
        return 1
    try:
        c.open()
    except Exception as exc:  # noqa: BLE001
        log(f"  !! could not open compass: {exc}")
        log("     (in 'i2c' group? check: i2cdetect -y -r 7  -> expect 0x0e)")
        _save()
        return 1

    log(f"WHO_AM_I = 0x{c.whoami:02x}  (expect 0x10)")
    if c.whoami != 0x10:
        log("  !! unexpected WHO_AM_I — is this really the IST8310?")
    log(f"calibration: {'LOADED ' + c.cal['path'] if c.is_calibrated else 'none (RAW)'}")
    log(f"\nrotate the car SLOWLY through a full circle now — reading {WINDOW:.0f}s:\n")

    headings, xs, ys = [], [], []
    try:
        end = time.monotonic() + WINDOW
        while time.monotonic() < end:
            r = c.read()
            headings.append(r["heading_deg"])
            xs.append(r["x"]); ys.append(r["y"])
            log(f"   X={r['x']:8.1f}  Y={r['y']:8.1f}  Z={r['z']:8.1f} uT   "
                f"heading={r['heading_deg']:6.1f} deg")
            time.sleep(0.25)
    finally:
        c.close()

    log("\n[SUMMARY]")
    log(f"   samples        : {len(headings)}")
    if headings:
        log(f"   heading range  : {min(headings):.0f}..{max(headings):.0f} deg "
            f"(swept {max(headings) - min(headings):.0f} deg)")
        log(f"   |X| span       : {max(xs) - min(xs):.1f} uT")
        log(f"   |Y| span       : {max(ys) - min(ys):.1f} uT")
    moved = headings and (max(headings) - min(headings)) > 30
    if c.whoami == 0x10 and headings:
        log("\n[RESULT] Compass WORKS — IST8310 confirmed and returning field data."
            + ("  Heading tracked your rotation." if moved else
               "  (Rotate the car to see the heading sweep.)"))
    else:
        log("\n[RESULT] Compass problem — see warnings above.")
        _save()
        return 1
    _save()
    return 0


def _save():
    with open(REPORT, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"\nreport -> {REPORT}")


if __name__ == "__main__":
    sys.exit(main())
