#!/usr/bin/env python3
"""
apps/test_oled.py — prove the SSD1306 OLED works (you confirm it visually).

Draws a test screen with a live counter for a few seconds. If the panel shows the
text, the OLED + I2C wiring are good. Nothing is captured — WATCH THE PANEL.

⚠️ If the oled_stats service is running it owns the panel and this will fight it.
Stop it first:   sudo systemctl stop oled_stats.service
(restart later:  sudo systemctl start oled_stats.service)

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_oled.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from drivers.oled import OLED   # noqa: E402

SECONDS = 8


def main():
    print(f"OLED test @ {time.strftime('%Y-%m-%d %H:%M:%S')}")
    dev = "/dev/i2c-7"
    if not os.path.exists(dev):
        print(f"  !! {dev} missing — I2C bus not present.")
        return 1
    try:
        oled = OLED().open()
    except Exception as exc:  # noqa: BLE001
        print(f"  !! could not open OLED: {exc}")
        print("     - luma.oled installed?  pip3 install --user luma.oled")
        print("     - panel detected?  i2cdetect -y -r 7  (expect 0x3c)")
        print("     - oled_stats.service holding it?  sudo systemctl stop oled_stats.service")
        return 1

    print(f"Drawing to the panel for {SECONDS}s — WATCH THE OLED. Ctrl-C to stop.")
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < SECONDS:
            t = time.monotonic() - t0
            oled.text([
                "robocar OLED OK",
                "SSD1306 i2c-7 0x3c",
                "-" * 20,
                f"uptime: {t:4.1f} s",
                "test pattern",
            ])
            time.sleep(0.2)
        oled.close()
    except KeyboardInterrupt:
        oled.close()
        print("\nstopped.")

    print("\n[RESULT] If you saw 'robocar OLED OK' + a counting timer on the panel,")
    print("         the OLED works. If the screen stayed blank, see the hints above.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
