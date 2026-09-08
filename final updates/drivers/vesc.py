#!/usr/bin/env python3
"""
drivers/vesc.py — safety-first VESC serial control (Flipsky Mini FSESC 6.7 Pro).

Ported from Camera Control/vesc_driver.py. Native VESC packet protocol over USB
(/dev/ttyACM0), resolved by stable by-id. Throttle (duty/current/rpm), steering-
servo output, and live telemetry (voltage, temps, rpm, tach, fault).

Safety:
  - The VESC stops the motor if it gets no command within ~1 s, so a controller
    must stream commands continuously; dropping the link = coast/stop.
  - stop() zeros current; it also runs on context-exit.
  - The VESC only enumerates while its MOTOR BATTERY is powered (logic is battery-
    fed, not USB). No battery => no /dev/ttyACM0.

Protocol: start(0x02,len<256), len, payload, CRC16-CCITT(0x1021,init0), 0x03.
"""
import os
import sys
import struct
import time
import serial

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # single source of truth

BAUD = config.VESC_BAUD

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_SET_HANDBRAKE = 10
COMM_SET_SERVO_POS = 12

# VESC fault codes (fw 5.x/6.x) for human-readable telemetry.
FAULT_NAMES = {
    0: "NONE", 1: "OVER_VOLTAGE", 2: "UNDER_VOLTAGE", 3: "DRV",
    4: "ABS_OVER_CURRENT", 5: "OVER_TEMP_FET", 6: "OVER_TEMP_MOTOR",
    7: "GATE_DRIVER_OVER_VOLTAGE", 8: "GATE_DRIVER_UNDER_VOLTAGE",
    9: "MCU_UNDER_VOLTAGE", 10: "BOOTING_FROM_WATCHDOG_RESET",
    11: "ENCODER_SPI", 12: "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE", 14: "FLASH_CORRUPTION",
}


def resolve_port():
    return config.vesc_port()


def _crc16(data):
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class VESC:
    def __init__(self, port=None, baud=BAUD, timeout=0.1):
        self.port = port or resolve_port()
        self.ser = serial.Serial(self.port, baud, timeout=timeout)

    # --- framing --------------------------------------------------------- #
    def _send(self, payload):
        crc = _crc16(payload)
        pkt = bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])
        self.ser.write(pkt)

    def _recv(self):
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

    # --- info / telemetry ------------------------------------------------ #
    def firmware(self):
        p = self._query(bytes([COMM_FW_VERSION]))
        if not p or len(p) < 3:
            return None
        hw = p[3:].split(b"\x00")[0].decode(errors="replace")
        return {"major": p[1], "minor": p[2], "hw": hw}

    def get_values(self, settle=0.05):
        # `settle` is how long to wait before reading the reply. The 0.05 default is
        # deliberately conservative (~20 Hz ceiling). _recv's read() blocks up to the
        # port timeout anyway, so a caller that needs a faster telemetry rate -- e.g.
        # apps/sysid.py measuring a step response -- can safely pass settle=0.005.
        p = self._query(bytes([COMM_GET_VALUES]), settle=settle)
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
                "fault": fault, "fault_name": FAULT_NAMES.get(fault, f"?{fault}")}

    # --- actuation ------------------------------------------------------- #
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
