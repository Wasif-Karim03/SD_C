#!/usr/bin/env python3
"""
apps/test_gps.py — prove the GPS (SE100 / u-blox M8N) works.

Reads NMEA on /dev/ttyTHS1 @ 38400 for a window and reports:
  - whether NMEA sentences are FLOWING (proves the UART wiring + GPS power/link),
  - which GNSS talkers are present (GP/GN/GL/GA...),
  - satellites in view / used, fix quality, and lat/lon if a fix is obtained.

IMPORTANT: indoors the GPS usually gets NO fix (status 'V', 0 sats) — that's
expected and NOT a failure. A working GPS still streams NMEA and reports
satellites in view. For an actual position fix, run it near a window or outside.

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 test_gps.py
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from drivers.gps import GPS   # noqa: E402

REPORT = os.path.join(HERE, "gps_report.txt")
WINDOW = 15.0

lines = []


def log(m=""):
    print(m)
    lines.append(m)


def main():
    log("=" * 64)
    log(f"GPS (SE100 / M8N) test @ {time.strftime('%Y-%m-%d %H:%M:%S')}")
    log("=" * 64)
    gps = GPS()
    log(f"port: {gps.port} @ {gps.baud}")
    if not os.path.exists(gps.port):
        log("  !! UART path missing — check the 40-pin header wiring.")
        _save()
        return 1
    try:
        gps.open()
    except Exception as exc:  # noqa: BLE001
        log(f"  !! could not open UART: {exc}")
        log("     This UART is likely held by ANOTHER PROCESS. Free it and retry:")
        log("       sudo fuser -v /dev/ttyTHS1        # see who holds it")
        log("       pkill -f gps_web.py               # leftover dashboard?")
        log("       sudo systemctl stop nvgetty       # serial console on the UART")
        log("       sudo systemctl stop serial-getty@ttyTHS1.service")
        _save()
        return 1

    log(f"\nreading NMEA for {WINDOW:.0f}s (first few lines shown) ...")
    shown = [0]

    def on_line(raw):
        if shown[0] < 8:
            log(f"   {raw}")
            shown[0] += 1

    try:
        st = gps.poll(WINDOW, on_sentence=on_line)
    finally:
        gps.close()

    log("\n[SUMMARY]")
    log(f"   NMEA lines read : {st['nmea_lines']}")
    log(f"   GNSS talkers    : {', '.join(st['talkers']) or 'none'}")
    log(f"   sats in view    : {st['sats_in_view']}")
    log(f"   sats used in fix: {st['sats_used']}")
    log(f"   fix quality     : {st['fix_quality']}  (0 = no fix)")
    log(f"   RMC status      : {st['rmc_status']}  (A = valid, V = void)")
    if st["has_fix"]:
        log(f"   POSITION FIX    : lat={st['lat']:.6f}  lon={st['lon']:.6f}"
            + (f"  alt={st['alt_m']:.1f} m" if st['alt_m'] is not None else ""))
        log("\n[RESULT] GPS fully working — streaming NMEA AND has a position fix.")
    elif st["nmea_lines"] > 0:
        log("\n[RESULT] GPS link WORKS — NMEA is flowing and it sees satellites.")
        log("         No position fix yet (normal indoors). Retry near a window")
        log("         / outside to confirm a lat/lon lock.")
    else:
        if st["read_errors"] > 10:
            log(f"\n[RESULT] NO NMEA — {st['read_errors']} read errors: another "
                "process/getty is holding /dev/ttyTHS1. Free it and retry:")
            log("   sudo fuser -v /dev/ttyTHS1")
            log("   pkill -f gps_web.py")
            log("   sudo systemctl stop nvgetty")
            log("   sudo systemctl stop serial-getty@ttyTHS1.service")
        else:
            log("\n[RESULT] NO NMEA received — check: baud 38400, TX/RX crossed "
                "(pins 8/10), GPS powered.")
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
