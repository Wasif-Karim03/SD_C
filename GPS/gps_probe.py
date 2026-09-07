#!/usr/bin/env python3
"""Probe a serial port for NMEA GPS sentences (Radiolink SE100 / u-blox M8N).

Tries the given port(s) at the given baud(s) and prints any NMEA lines seen.
Usage: python3 gps_probe.py [port] [baud] [seconds]
"""
import sys
import time
import serial

PORTS = [sys.argv[1]] if len(sys.argv) > 1 else ["/dev/ttyTHS1", "/dev/ttyTHS2"]
BAUDS = [int(sys.argv[2])] if len(sys.argv) > 2 else [9600, 38400, 115200, 57600, 4800]
WINDOW = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0


def looks_like_nmea(line: bytes) -> bool:
    return line.startswith(b"$") and b"," in line and (b"GP" in line or b"GN" in line or b"GL" in line or b"GA" in line)


def probe(port: str, baud: float) -> bool:
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        print(f"  [{port} @ {baud}] open failed: {e}")
        return False
    found = False
    raw_seen = False
    deadline = time.time() + WINDOW
    buf = b""
    try:
        while time.time() < deadline:
            chunk = ser.read(256)
            if chunk:
                raw_seen = True
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if looks_like_nmea(line):
                        print(f"  [{port} @ {baud}] NMEA: {line.decode(errors='replace')}")
                        found = True
    finally:
        ser.close()
    if not found and raw_seen:
        print(f"  [{port} @ {baud}] got bytes but no clean NMEA (wrong baud?)")
    elif not found:
        print(f"  [{port} @ {baud}] silent")
    return found


def main():
    for port in PORTS:
        print(f"== {port} ==")
        for baud in BAUDS:
            if probe(port, baud):
                print(f"\n>>> GPS FOUND on {port} @ {baud} baud\n")
                return
    print("\n>>> No NMEA stream detected on any port/baud tried.")


if __name__ == "__main__":
    main()
