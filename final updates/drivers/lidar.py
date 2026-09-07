#!/usr/bin/env python3
"""
drivers/lidar.py — clean, self-contained RPLIDAR C1 driver (no external lidar libs).

Ported from the proven LiDAR/rplidar_c1.py. Only needs pyserial. Talks the RPLIDAR
scan protocol directly:
  - connect / disconnect / reset / stop
  - get_info(), get_health()
  - iter_scans()   -> yields full 360deg revolutions as lists of (quality, angle_deg, dist_mm)
  - grab_scans(n)  -> convenience: return the next n full scans as a list

Hardware (fixed, verified on-car):
  RPLIDAR C1, Silicon Labs CP2102N USB bridge, 460800 baud, MODEL 0x41, FW 1.01.
  Pinned by STABLE by-id path (survives USB re-enumeration; /dev/ttyUSBn can swap).

DTR is held low on open so the C1's motor isn't held in reset (it spins whenever
powered). One process at a time may own the port.
"""
import os
import sys
import time
import threading
import serial

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # single source of truth

BY_ID = config.by_id_path(config.LIDAR_BY_ID)
FALLBACK = config.LIDAR_FALLBACK
BAUD = config.LIDAR_BAUD

SYNC = 0xA5
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
DESCRIPTOR_LEN = 7


class RPLidarC1:
    def __init__(self, port=None, baud=BAUD, timeout=3):
        import os
        self.port = port or (BY_ID if os.path.exists(BY_ID) else FALLBACK)
        self.baud = baud
        self.timeout = timeout
        self._ser = None

    # ---- low level ------------------------------------------------------
    def connect(self):
        if self._ser:
            self.disconnect()
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        try:
            self._ser.setDTR(False)   # don't hold the motor in reset
        except Exception:
            pass
        return self

    def disconnect(self):
        if self._ser:
            try:
                self.stop()
            except Exception:
                pass
            self._ser.close()
            self._ser = None

    def _send(self, cmd, payload=b""):
        req = bytes([SYNC, cmd])
        if payload:
            req += bytes([len(payload)]) + payload
            req += bytes([(sum(req[1:]) & 0xFF)])
        self._ser.write(req)

    def _read_descriptor(self):
        d = self._ser.read(DESCRIPTOR_LEN)
        if len(d) != DESCRIPTOR_LEN or d[0] != 0xA5 or d[1] != 0x5A:
            raise IOError(
                f"bad/empty descriptor ({d.hex()!r}); port busy (another lidar "
                "process running?), wrong baud, or device unpowered")
        length = d[2] | (d[3] << 8) | (d[4] << 16) | ((d[5] & 0x3F) << 24)
        return length, d[6]

    # ---- commands -------------------------------------------------------
    def stop(self):
        self._send(CMD_STOP)
        time.sleep(0.02)
        self._ser.reset_input_buffer()

    def reset(self):
        self._send(CMD_RESET)
        time.sleep(0.8)
        self._ser.reset_input_buffer()

    def get_info(self):
        self._ser.reset_input_buffer()
        self._send(CMD_GET_INFO)
        length, _ = self._read_descriptor()
        raw = self._ser.read(length)
        if len(raw) < 20:
            raise IOError("short info payload; port busy or device unpowered")
        model, fw_minor, fw_major, hw = raw[0], raw[1], raw[2], raw[3]
        serialnum = raw[4:20][::-1].hex().upper()
        return {"model": model, "model_hex": f"0x{model:02X}",
                "firmware": f"{fw_major}.{fw_minor:02d}",
                "hardware": hw, "serial": serialnum}

    def get_health(self):
        self._ser.reset_input_buffer()
        self._send(CMD_GET_HEALTH)
        length, _ = self._read_descriptor()
        raw = self._ser.read(length)
        if len(raw) < 3:
            raise IOError("short health payload; port busy or device unpowered")
        status = raw[0]
        errcode = raw[1] | (raw[2] << 8)
        return {"status": status,
                "status_str": ["Good", "Warning", "Error"][status] if status < 3 else "?",
                "error_code": errcode}

    # ---- scanning -------------------------------------------------------
    def _iter_measurements(self):
        # BULK read: pull everything waiting in the OS buffer and parse it
        # in-process, instead of one 5-byte serial read per point. The per-point
        # read() was the bottleneck (~2 scans/s); bulk parsing hits the sensor's
        # real ~8-10 Hz.
        self._ser.reset_input_buffer()
        self._send(CMD_SCAN)
        self._read_descriptor()
        buf = bytearray()
        while True:
            waiting = self._ser.in_waiting
            chunk = self._ser.read(waiting if waiting > 0 else 5)
            if not chunk:
                continue
            buf.extend(chunk)
            i, ln = 0, len(buf)
            while ln - i >= 5:
                b0, b1 = buf[i], buf[i + 1]
                start = b0 & 0x01
                inv_start = (b0 >> 1) & 0x01
                check = b1 & 0x01
                if start == inv_start or check != 1:
                    i += 1              # desync -> slide one byte
                    continue
                b2, b3, b4 = buf[i + 2], buf[i + 3], buf[i + 4]
                quality = b0 >> 2
                angle = ((b1 >> 1) | (b2 << 7)) / 64.0
                dist = (b3 | (b4 << 8)) / 4.0
                yield bool(start), quality, angle, dist
                i += 5
            del buf[:i]                 # keep the unparsed remainder

    def iter_scans(self, min_points=90):
        """Yield full revolutions: lists of (quality, angle_deg, dist_mm).
        Points with dist==0 (no return) are dropped."""
        scan = []
        for new_scan, quality, angle, dist in self._iter_measurements():
            if new_scan and scan:
                if len(scan) >= min_points:
                    yield scan
                scan = []
            if dist > 0:
                scan.append((quality, angle, dist))

    def grab_scans(self, n=5, min_points=90):
        """Return the next n full scans (blocks until collected)."""
        out = []
        for scan in self.iter_scans(min_points=min_points):
            out.append(scan)
            if len(out) >= n:
                break
        return out

    def __enter__(self):
        return self.connect()

    def __exit__(self, *a):
        self.disconnect()


class ThreadedLidar:
    """Background reader that always exposes the NEWEST full scan (non-blocking).

    Like ThreadedCamera but for the LiDAR: a daemon thread spins iter_scans() and
    keeps only the latest revolution, so a control loop can grab the freshest scan
    without blocking on a full rotation.

        lid = ThreadedLidar().start()
        scan, age = lid.latest()     # scan = [(quality, angle_deg, dist_mm), ...]
        lid.stop()
    """

    def __init__(self, port=None, min_points=120):
        self.lidar = RPLidarC1(port=port)
        self.min_points = min_points
        self._scan = None
        self._t = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self.info = None
        self.health = None
        # per-bearing self-return mask (the car's own body); 0 = no mask
        self._mask = config.load_self_mask() or [0.0] * 360

    def start(self):
        self.lidar.connect()
        try:
            self.info = self.lidar.get_info()
            self.health = self.lidar.get_health()
        except Exception:
            pass
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        t0 = time.monotonic()
        while self._scan is None and time.monotonic() - t0 < 5.0:
            time.sleep(0.01)
        if self._scan is None:
            print("  [ThreadedLidar] WARNING: connected but no scans in 5s — the "
                  "port is likely held by another process (pkill it / fuser -k "
                  "/dev/ttyUSB0) or the motor isn't spinning.")
        return self

    def _loop(self):
        try:
            masked = any(self._mask)
            for scan in self.lidar.iter_scans(min_points=self.min_points):
                if not self._running:
                    break
                if masked:
                    scan = [p for p in scan
                            if p[2] / 1000.0 > self._mask[int(p[1]) % 360]]
                with self._lock:
                    self._scan = scan
                    self._t = time.monotonic()
        except Exception:
            self._running = False

    def latest(self):
        """Return (scan, age_seconds) — newest full revolution, or (None, inf)."""
        with self._lock:
            if self._scan is None:
                return None, float("inf")
            return self._scan, time.monotonic() - self._t

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.lidar.disconnect()

    def __enter__(self):
        return self.start()

    def __exit__(self, *a):
        self.stop()
