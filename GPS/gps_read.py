#!/usr/bin/env python3
"""Continuous NMEA reader for the Radiolink SE100 on the Jetson header UART.

Usage: python3 gps_read.py [port] [baud]
Defaults: /dev/ttyTHS1 @ 38400  (Radiolink SE100 ships at 38400, not 9600)

Prints raw NMEA sentences and a decoded fix summary (lat/lon/sats) from $..GGA.
Ctrl-C to stop.
"""
import sys
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 38400


def dm_to_deg(val: str, hemi: str):
    """Convert NMEA ddmm.mmmm to signed decimal degrees."""
    if not val:
        return None
    dot = val.find(".")
    deg_len = dot - 2
    deg = float(val[:deg_len])
    minutes = float(val[deg_len:])
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Reading {PORT} @ {BAUD} ... (Ctrl-C to stop)")
    try:
        while True:
            raw = ser.readline().decode(errors="replace").strip()
            if not raw.startswith("$"):
                continue
            print(raw)
            f = raw.split(",")
            if f[0].endswith("GGA") and len(f) >= 8:
                lat = dm_to_deg(f[2], f[3])
                lon = dm_to_deg(f[4], f[5])
                fix = f[6]
                sats = f[7]
                if fix not in ("", "0") and lat is not None:
                    print(f"   --> FIX: lat={lat:.6f} lon={lon:.6f} sats={sats}")
                else:
                    print(f"   --> no fix yet (sats in view={sats})")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
