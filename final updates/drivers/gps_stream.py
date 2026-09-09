#!/usr/bin/env python3
"""
drivers/gps_stream.py — a continuously-running NMEA reader for the cockpit.

drivers/gps.py already talks to the SE100, but its poll() blocks for a window,
which is right for a test script and wrong for a control loop that must never
stall. This wraps the same port in a thread and keeps ONE snapshot dict that any
loop can read without waiting.

What it decodes and why:
  GGA  fix quality, satellites used, HDOP, altitude, position
  RMC  course and speed over ground (the only heading this car has, since the
       compass is uncalibrated and there is no IMU)
  GSV  per-satellite C/N0 — a satellite COUNT is nearly useless on its own;
       four satellites at 18 dB-Hz and four at 42 dB-Hz are the same number
       and completely different situations. The cockpit draws the bars.

Design rules, both learned the hard way elsewhere in this codebase:
  · This never raises into its caller. A GPS that stops talking must not be
    able to take the drive loop down with it.
  · A fix goes stale. If no GGA has arrived for FIX_TTL_S the snapshot reports
    fix=False rather than serving the last known position forever. A position
    that is quietly two minutes old is worse than no position, because the
    operator will act on it.
"""
import math
import threading
import time

FIX_TTL_S = 3.0          # a fix older than this is not a fix
SAT_TTL_S = 12.0         # GSV cycles every few seconds; be generous


def _dm_to_deg(val, hemi):
    """NMEA ddmm.mmmm + hemisphere -> signed decimal degrees."""
    if not val:
        return None
    dot = val.find(".")
    if dot < 3:
        return None
    try:
        deg = float(val[:dot - 2])
        minutes = float(val[dot - 2:])
    except ValueError:
        return None
    dec = deg + minutes / 60.0
    return -dec if hemi in ("S", "W") else dec


def _f(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _checksum_ok(line):
    """NMEA XOR checksum. Cheap, and it is the only thing standing between a
    noisy 38400 UART and a position that is off by a degree."""
    if "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


class ThreadedGPS:
    def __init__(self, port=None, baud=None):
        import config
        self.port = port or config.GPS_PORT
        self.baud = baud or config.GPS_BAUD
        self.ser = None
        self.running = False
        self.thread = None
        self.read_errors = 0
        self.sentences = 0
        self._lock = threading.Lock()
        self._s = {"fix": False, "lat": None, "lon": None, "alt": None,
                   "sats": 0, "hdop": None, "mode": None,
                   "course": None, "speed": None, "t": 0.0}
        self._sats = {}          # prn -> {"cn": int, "used": bool, "t": float}
        self._used = set()

    # ------------------------------------------------------------------ #

    def start(self):
        try:
            import serial
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0,
                                     exclusive=True)
        except Exception as e:                                # noqa: BLE001
            # Not fatal. The car drives indoors without GNSS every day; the
            # cockpit shows ABSENT and the operator knows exactly where it is.
            print("  [gps] not available:", e)
            self.ser = None
            return self
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f"  [gps] streaming {self.port} @ {self.baud}")
        return self

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.5)
        if self.ser:
            try:
                self.ser.close()
            except Exception:                                 # noqa: BLE001
                pass

    # ------------------------------------------------------------------ #

    def _loop(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except Exception:                                 # noqa: BLE001
                self.read_errors += 1
                time.sleep(0.2)
                continue
            if not raw:
                continue
            try:
                line = raw.decode("ascii", "ignore").strip()
            except Exception:                                 # noqa: BLE001
                continue
            if not line.startswith("$") or not _checksum_ok(line):
                continue
            self.sentences += 1
            try:
                self._parse(line)
            except Exception:                                 # noqa: BLE001
                # One malformed sentence must never end the stream.
                pass

    def _parse(self, line):
        f = line.split("*")[0].split(",")
        talker, kind = f[0][1:3], f[0][3:]
        now = time.monotonic()

        if kind == "GGA" and len(f) >= 10:
            quality = f[6]
            fix = quality not in ("", "0")
            with self._lock:
                if fix:
                    self._s.update({
                        "fix": True,
                        "lat": _dm_to_deg(f[2], f[3]),
                        "lon": _dm_to_deg(f[4], f[5]),
                        "sats": int(f[7]) if f[7].isdigit() else 0,
                        "hdop": _f(f[8]),
                        "alt": _f(f[9]),
                        "mode": {"1": "GPS", "2": "DGPS", "4": "RTK FIX",
                                 "5": "RTK FLOAT", "6": "DEAD REC"}.get(quality, "FIX"),
                        "t": now,
                    })
                else:
                    self._s["fix"] = False
                    self._s["sats"] = int(f[7]) if f[7].isdigit() else 0
                    self._s["t"] = now

        elif kind == "RMC" and len(f) >= 9:
            with self._lock:
                # course over ground only exists while moving; a parked receiver
                # reports garbage here, so it is left as None below 0.3 m/s
                spd = _f(f[7])
                self._s["speed"] = None if spd is None else spd * 0.514444
                cog = _f(f[8])
                self._s["course"] = cog if (spd is not None and spd > 0.6) else None

        elif kind == "GSA" and len(f) >= 15:
            used = set()
            for i in range(3, 15):
                if i < len(f) and f[i]:
                    used.add(talker + f[i])
            with self._lock:
                self._used = used

        elif kind == "GSV" and len(f) >= 4:
            # 4 fields per satellite: prn, elevation, azimuth, C/N0
            i = 4
            with self._lock:
                while i + 3 < len(f):
                    prn = f[i]
                    cn = f[i + 3]
                    if prn:
                        key = talker + prn
                        self._sats[key] = {
                            "prn": key,
                            "cn": int(cn) if cn.isdigit() else 0,
                            "el": _f(f[i + 1]),
                            "az": _f(f[i + 2]),
                            "t": now,
                        }
                    i += 4

    # ------------------------------------------------------------------ #

    def snapshot(self):
        """A plain dict, safe to hand straight to json.dumps."""
        now = time.monotonic()
        with self._lock:
            s = dict(self._s)
            sats = [dict(v) for v in self._sats.values() if now - v["t"] < SAT_TTL_S]
            used = set(self._used)
        if s["t"] and (now - s["t"]) > FIX_TTL_S:
            # The receiver stopped talking. Say so; do not serve a ghost.
            s["fix"] = False
            s["stale"] = round(now - s["t"], 1)
        for v in sats:
            v["used"] = v["prn"] in used
            v.pop("t", None)
        sats.sort(key=lambda d: (-d["cn"], d["prn"]))
        s["satlist"] = sats[:24]
        s["present"] = self.ser is not None
        s["errors"] = self.read_errors
        return s
