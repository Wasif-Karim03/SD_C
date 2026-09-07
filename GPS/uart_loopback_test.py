#!/usr/bin/env python3
"""UART loopback test for the Jetson 40-pin header UART (ttyTHS1, pins 8 & 10).

PURPOSE: prove the Jetson UART hardware/port works, independent of the GPS.

WIRING FOR THIS TEST:
  1. Disconnect the SE100's orange (pin 8) and white (pin 10) wires.
  2. Jumper Jetson pin 8 (UART TX) directly to pin 10 (UART RX).
  3. Run:  python3 uart_loopback_test.py
  4. Re-wire the GPS afterwards.

PASS  -> the UART port is good; the fault is on the GPS side (swapped TX/RX,
         loose pin, or GPS not transmitting).
FAIL  -> the port itself isn't sending/receiving; pin-mux or hardware issue.
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

ser = serial.Serial(PORT, BAUD, timeout=1)
ser.reset_input_buffer()
ser.reset_output_buffer()

msg = b"JETSON-UART-LOOPBACK-OK\r\n"
print(f"Writing test pattern on {PORT} @ {BAUD} ...")
ser.write(msg)
ser.flush()
time.sleep(0.3)
echo = ser.read(len(msg) + 8)
ser.close()

if msg.strip() in echo:
    print(f"PASS: received back -> {echo!r}")
    print(">>> UART port works. The GPS silence is a wiring/power issue on the SE100 side.")
    sys.exit(0)
else:
    print(f"FAIL: got -> {echo!r}")
    print(">>> Nothing looped back. Check the pin8<->pin10 jumper, or pin-mux.")
    sys.exit(1)
