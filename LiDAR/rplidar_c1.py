"""
Minimal, self-contained RPLIDAR C1 driver (no external lidar libs).

Only needs pyserial. Implements just what we need to test data reading:
  - connect / reset / stop
  - get_health, get_info
  - iter_scans() -> yields full 360deg revolutions as lists of (quality, angle_deg, dist_mm)

Protocol verified on-car: CP2102N bridge @ 460800 baud, MODEL 0x41 (C1), FW 1.01.
Stable device path (survives USB re-enumeration):
  /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_cad4bd81365aee11899081dc8ffcc75d-if00-port0
"""
import time
import struct
import serial

DEFAULT_PORT = ("/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_"
                "Controller_cad4bd81365aee11899081dc8ffcc75d-if00-port0")
BAUD = 460800

# command bytes (prefixed with sync 0xA5)
SYNC = 0xA5
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52

DESCRIPTOR_LEN = 7


class RPLidarC1:
    def __init__(self, port=DEFAULT_PORT, baud=BAUD, timeout=3):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser = None

    # ---- low level ------------------------------------------------------
    def connect(self):
        if self._ser:
            self.disconnect()
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        # C1 motor spins whenever powered; DTR low keeps it from being held in reset
        try:
            self._ser.setDTR(False)
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
            req += bytes([(sum(req[1:]) & 0xFF)])  # checksum for payload cmds
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
        return {"model": model, "firmware": f"{fw_major}.{fw_minor:02d}",
                "hardware": hw, "serial": serialnum}

    def get_health(self):
        self._ser.reset_input_buffer()
        self._send(CMD_GET_HEALTH)
        length, _ = self._read_descriptor()
        raw = self._ser.read(length)
        if len(raw) < 3:
            raise IOError("short health payload; port busy or device unpowered")
        status = raw[0]  # 0=Good 1=Warning 2=Error
        errcode = raw[1] | (raw[2] << 8)
        return {"status": status,
                "status_str": ["Good", "Warning", "Error"][status] if status < 3 else "?",
                "error_code": errcode}

    # ---- scanning -------------------------------------------------------
    def _iter_measurements(self):
        """Yield (new_scan, quality, angle_deg, dist_mm) for each 5-byte node."""
        self._ser.reset_input_buffer()
        self._send(CMD_SCAN)
        self._read_descriptor()  # (5, single, data-mode)
        while True:
            b = self._ser.read(5)
            if len(b) != 5:
                continue
            b0, b1, b2, b3, b4 = b
            start = b0 & 0x01
            inv_start = (b0 >> 1) & 0x01
            check = b1 & 0x01
            if start == inv_start or check != 1:
                # desync -> resync by scanning forward one byte
                self._ser.read(1)
                continue
            quality = b0 >> 2
            angle = ((b1 >> 1) | (b2 << 7)) / 64.0
            dist = (b3 | (b4 << 8)) / 4.0
            yield bool(start), quality, angle, dist

    def iter_scans(self, min_points=90):
        """Yield full revolutions as lists of (quality, angle_deg, dist_mm).
        Points with dist==0 (no return) are dropped."""
        scan = []
        for new_scan, quality, angle, dist in self._iter_measurements():
            if new_scan and scan:
                if len(scan) >= min_points:
                    yield scan
                scan = []
            if dist > 0:
                scan.append((quality, angle, dist))
