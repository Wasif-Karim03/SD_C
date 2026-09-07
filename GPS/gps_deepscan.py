#!/usr/bin/env python3
"""Exhaustive, non-invasive scan: every port x every plausible baud, longer
window, and a RAW hexdump of anything received.

Logic:
  - clean NMEA        -> GPS works, we found the baud
  - garbage bytes     -> WIRE IS GOOD, baud just wrong (very fixable)
  - total silence     -> no signal reaching the Jetson RX pin (wiring/power/mux)
"""
import time
import serial

PORTS = ["/dev/ttyTHS1", "/dev/ttyTHS2"]
BAUDS = [9600, 38400, 57600, 115200, 4800, 19200, 230400, 460800, 921600]
WINDOW = 3.0


def scan(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.4)
    except Exception as e:
        return ("err", str(e))
    raw = b""
    deadline = time.time() + WINDOW
    try:
        while time.time() < deadline:
            chunk = ser.read(512)
            if chunk:
                raw += chunk
    finally:
        ser.close()
    if not raw:
        return ("silent", b"")
    if b"$G" in raw and b"," in raw:
        return ("nmea", raw)
    return ("garbage", raw)


def main():
    any_bytes = False
    for port in PORTS:
        print(f"\n===== {port} =====")
        for baud in BAUDS:
            kind, data = scan(port, baud)
            if kind == "nmea":
                lines = [l for l in data.split(b"\n") if l.startswith(b"$")][:3]
                print(f"  {baud:>7}: NMEA! e.g. {lines}")
                print(f"\n>>> GPS FOUND on {port} @ {baud}\n")
                return
            elif kind == "garbage":
                any_bytes = True
                print(f"  {baud:>7}: {len(data)} raw bytes (baud likely wrong) "
                      f"hex={data[:16].hex()}")
            elif kind == "err":
                print(f"  {baud:>7}: open error: {data}")
            else:
                print(f"  {baud:>7}: silent")
    print()
    if any_bytes:
        print(">>> Got RAW BYTES but no NMEA -> the wire is connected; it's a baud/"
              "config mismatch. Solvable in software. Tell me the hex above.")
    else:
        print(">>> TOTAL SILENCE on every port/baud -> nothing is reaching the "
              "Jetson RX pin. That's wiring/power/pin-mux, not baud.")


if __name__ == "__main__":
    main()
