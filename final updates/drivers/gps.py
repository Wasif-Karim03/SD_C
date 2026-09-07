#!/usr/bin/env python3
"""
drivers/gps.py — Radiolink SE100 (u-blox M8N) NMEA reader on the Jetson header UART.

Ported from GPS/gps_read.py. Reads NMEA on /dev/ttyTHS1 @ 38400 (pins 8/10, TX/RX
crossed) and decodes a live fix. Not a USB device -> plain UART path, no by-id.

  read_sentence()       -> one raw NMEA line (str) or None (also None on a transient
                           read hiccup — counts it in .read_errors)
  poll(seconds)         -> aggregate state (fix, sats, lat/lon, ...) over a window

Robustness: opens the port EXCLUSIVE so a second holder (a leftover gps_web.py, or
the serial-console getty on this UART) fails fast with a clear message instead of
the cryptic "device reports readiness to read but returned no data". Transient read
errors are swallowed and counted so one hiccup can't crash a run.
"""
import os
import sys
import time
import serial

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # single source of truth

PORT = config.GPS_PORT
BAUD = config.GPS_BAUD   # SE100 ships at 38400, NOT the u-blox 9600 default


def dm_to_deg(val, hemi):
    """NMEA ddmm.mmmm + hemisphere -> signed decimal degrees."""
    if not val:
        return None
    dot = val.find(".")
    if dot < 3:
        return None
    deg = float(val[:dot - 2])
    minutes = float(val[dot - 2:])
    dec = deg + minutes / 60.0
    return -dec if hemi in ("S", "W") else dec


class GPS:
    def __init__(self, port=PORT, baud=BAUD, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.read_errors = 0

    def open(self):
        if self.ser is None:
            try:
                # exclusive=True -> if another process already holds this UART,
                # fail here with EBUSY instead of racing reads later.
                self.ser = serial.Serial(self.port, self.baud,
                                         timeout=self.timeout, exclusive=True)
            except TypeError:
                # very old pyserial without the exclusive kwarg
                self.ser = serial.Serial(self.port, self.baud,
                                         timeout=self.timeout)
            self.ser.reset_input_buffer()
        return self

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def read_sentence(self):
        try:
            raw = self.ser.readline().decode(errors="replace").strip()
        except serial.SerialException:
            # transient "readiness but no data" hiccup — count and skip
            self.read_errors += 1
            time.sleep(0.05)
            return None
        return raw if raw.startswith("$") else None

    def poll(self, seconds=15.0, on_sentence=None):
        """Read for `seconds`, aggregate the latest fix info. Returns a dict."""
        state = {"nmea_lines": 0, "talkers": set(), "fix_quality": 0,
                 "sats_used": None, "sats_in_view": None, "lat": None,
                 "lon": None, "alt_m": None, "rmc_status": None,
                 "speed_knots": None, "course_deg": None, "has_fix": False,
                 "read_errors": 0}
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            raw = self.read_sentence()
            # Bail early if the port only throws errors and never yields a line
            # (classic "another process/getty owns this UART").
            if self.read_errors > 60 and state["nmea_lines"] == 0:
                break
            if not raw:
                continue
            state["nmea_lines"] += 1
            if on_sentence:
                on_sentence(raw)
            f = raw.split(",")
            tag = f[0][3:] if len(f[0]) >= 6 else f[0]
            state["talkers"].add(f[0][1:3] if len(f[0]) >= 3 else "?")
            try:
                if tag == "GGA" and len(f) >= 10:
                    state["fix_quality"] = int(f[6] or 0)
                    state["sats_used"] = int(f[7]) if f[7] else state["sats_used"]
                    lat, lon = dm_to_deg(f[2], f[3]), dm_to_deg(f[4], f[5])
                    if state["fix_quality"] > 0 and lat is not None:
                        state.update(lat=lat, lon=lon, has_fix=True,
                                     alt_m=float(f[9]) if f[9] else None)
                elif tag == "RMC" and len(f) >= 8:
                    state["rmc_status"] = f[2] or state["rmc_status"]
                    if f[7]:
                        state["speed_knots"] = float(f[7])
                    if len(f) > 8 and f[8]:
                        state["course_deg"] = float(f[8])
                elif tag == "GSV" and len(f) >= 4 and f[3]:
                    state["sats_in_view"] = int(f[3])
            except (ValueError, IndexError):
                pass
        state["talkers"] = sorted(state["talkers"])
        state["read_errors"] = self.read_errors
        return state

    def __enter__(self):
        return self.open()

    def __exit__(self, *a):
        self.close()
