#!/usr/bin/env python3
"""
vesc_driver.py — minimal, safety-first VESC serial control for the Jetson.

Talks the native VESC packet protocol over /dev/ttyACM0 (USB) or a UART. Lets the
Jetson command the FSESC 6.7: throttle (current/duty/erpm) for the drive motor and
servo position for the steering, plus read live telemetry (voltage, temps, rpm,
fault). This is the link the autonomous controller will use.

Safety notes:
  - The VESC stops the motor if it receives no command within its timeout (~1 s),
    so a controller must send commands continuously; dropping the link = coast/stop.
  - stop() commands zero current. Always call it (and it runs on context-exit).
  - Steering servo is 0.0..1.0 (0.5 = center). Throttle helpers take real units.

Protocol: start(0x02 for len<256), len, payload, CRC16-CCITT(0x1021,init0), 0x03.
"""

import struct
import time

import serial

# VESC COMM command ids (fw 5.x / 6.x).
COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_SET_HANDBRAKE = 10
COMM_SET_SERVO_POS = 12


def _crc16(data):
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class VESC:
    def __init__(self, port="/dev/ttyACM0", baud=115200, timeout=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.port = port

    # --- framing ---------------------------------------------------------- #
    def _send(self, payload):
        crc = _crc16(payload)
        pkt = bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])
        self.ser.write(pkt)

    def _recv(self):
        """Read one response packet; return payload bytes or None."""
        start = self.ser.read(1)
        if not start:
            return None
        if start[0] == 0x02:
            ln = self.ser.read(1)
            if not ln:
                return None
            length = ln[0]
        elif start[0] == 0x03:
            hl = self.ser.read(2)
            if len(hl) < 2:
                return None
            length = (hl[0] << 8) | hl[1]
        else:
            return None
        payload = self.ser.read(length)
        self.ser.read(3)  # crc(2) + stop(1)
        return payload if len(payload) == length else None

    def _query(self, payload, settle=0.05):
        self.ser.reset_input_buffer()
        self._send(payload)
        time.sleep(settle)
        return self._recv()

    # --- info / telemetry ------------------------------------------------- #
    def firmware(self):
        p = self._query(bytes([COMM_FW_VERSION]))
        if not p or len(p) < 3:
            return None
        hw = p[3:].split(b"\x00")[0].decode(errors="replace")
        return {"major": p[1], "minor": p[2], "hw": hw}

    def get_values(self):
        """Live telemetry. Field offsets per fw 5.x COMM_GET_VALUES layout."""
        p = self._query(bytes([COMM_GET_VALUES]))
        if not p or p[0] != COMM_GET_VALUES or len(p) < 30:
            return None
        i = 1

        def i16():
            nonlocal i
            v = struct.unpack_from(">h", p, i)[0]; i += 2; return v

        def i32():
            nonlocal i
            v = struct.unpack_from(">i", p, i)[0]; i += 4; return v

        temp_mos = i16() / 10.0
        temp_motor = i16() / 10.0
        motor_current = i32() / 100.0
        input_current = i32() / 100.0
        i32(); i32()                      # id, iq (skip)
        duty = i16() / 1000.0
        erpm = i32()
        v_in = i16() / 10.0
        amp_hours = i32() / 10000.0
        i32()                              # amp_hours_charged
        i32(); i32()                       # watt_hours, watt_hours_charged
        tach = i32()
        tach_abs = i32()
        fault = p[i] if i < len(p) else -1
        return {"temp_mos": temp_mos, "temp_motor": temp_motor,
                "motor_current": motor_current, "input_current": input_current,
                "duty": duty, "erpm": erpm, "v_in": v_in,
                "amp_hours": amp_hours, "tach": tach, "tach_abs": tach_abs,
                "fault": fault}

    # --- actuation -------------------------------------------------------- #
    def set_current(self, amps):
        self._send(bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(amps * 1000)))

    def set_brake_current(self, amps):
        self._send(bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack(">i", int(amps * 1000)))

    def set_duty(self, duty):
        duty = max(-0.95, min(0.95, duty))
        self._send(bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000)))

    def set_rpm(self, erpm):
        self._send(bytes([COMM_SET_RPM]) + struct.pack(">i", int(erpm)))

    def set_handbrake(self, amps):
        self._send(bytes([COMM_SET_HANDBRAKE]) + struct.pack(">i", int(amps * 1000)))

    def set_servo(self, pos):
        """Steering: 0.0 (full one way) .. 0.5 (center) .. 1.0 (full other)."""
        pos = max(0.0, min(1.0, pos))
        self._send(bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", int(pos * 1000)))

    def stop(self):
        self.set_current(0.0)

    def close(self):
        try:
            self.stop()
        finally:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
