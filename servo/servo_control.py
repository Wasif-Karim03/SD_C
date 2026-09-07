#!/usr/bin/env python3
"""
servo_control.py — drive the steering servo through the Arduino Nano.

Path: Jetson  --USB serial-->  Arduino Nano (arduino_servo.ino)  --D9-->  servo.
This script just sends angle commands and prints what the Nano echoes back, so
it's the quickest way to answer "can we move the servo at all?".

Usage:
  python3 servo_control.py                 # interactive: type 0..180, c, s, q
  python3 servo_control.py 120             # set 120 deg once and exit
  python3 servo_control.py center          # center (90) once
  python3 servo_control.py sweep           # run one min->max->center sweep
  python3 servo_control.py test            # quick L/center/R/center check
  python3 servo_control.py steer -1        # full left  via normalized -1..+1
  python3 servo_control.py steer 0.5       # half right

Options:
  --port /dev/ttyUSB0   force the serial port (default: auto-detect)
  --baud 115200         must match the sketch (default 115200)

Auto-detect prefers /dev/ttyUSB* (the Nano's USB-serial chip) and skips
/dev/ttyACM0 (that's the VESC). If detection guesses wrong, pass --port.

For the autonomy code, import ServoController instead of shelling out:

    from servo_control import ServoController
    with ServoController() as srv:     # opens + waits for READY
        srv.steer(-0.3)                # normalized: -1 full left .. +1 full right
        srv.center()
    # leaving the 'with' block detaches + closes the port

steer() maps -1..+1 onto the bench-tested LEFT_LIMIT..RIGHT_LIMIT range
(asymmetric — the linkage swings further one way than the other), matching the
sketch's safe clamp, so a runaway perception value can never command the
linkage past the point where the wheel connector binds.
"""
import sys
import glob
import time
import argparse

try:
    import serial
except ImportError:
    sys.exit("pyserial missing — install with: pip3 install --user pyserial")

VESC_PORT = "/dev/ttyACM0"   # don't auto-pick this; it's the motor controller

CENTER = 90
# Bench-tested safe steering range — the wheel connector binds outside this, so
# never command past it. Asymmetric: the linkage reaches further left than right.
# Keep these in sync with STEER_MIN / STEER_MAX in arduino_servo.ino.
LEFT_LIMIT = 60             # most-left  safe angle (norm = -1)
RIGHT_LIMIT = 115           # most-right safe angle (norm = +1)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def norm_to_angle(norm):
    """Map a normalized steer value (-1 left .. +1 right) to a servo angle.

    The two sides have different travel, so each half of the range is scaled
    independently: norm 0 -> CENTER, -1 -> LEFT_LIMIT, +1 -> RIGHT_LIMIT.
    """
    norm = clamp(float(norm), -1.0, 1.0)
    if norm < 0:
        angle = CENTER + norm * (CENTER - LEFT_LIMIT)
    else:
        angle = CENTER + norm * (RIGHT_LIMIT - CENTER)
    return int(round(angle))


def find_port():
    """Best guess at the Arduino's port: a ttyUSB*, else a non-VESC ttyACM*."""
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    if usb:
        return usb[0]
    acm = [p for p in sorted(glob.glob("/dev/ttyACM*")) if p != VESC_PORT]
    if acm:
        return acm[0]
    return None


def connect(port, baud):
    """Open the port and wait for the Nano to finish its auto-reset + READY."""
    ser = serial.Serial(port, baud, timeout=1)
    # Opening the port toggles DTR, which resets the Nano. Give the bootloader
    # a moment, then drain its "READY ..." banner.
    time.sleep(2.0)
    ser.reset_input_buffer()
    deadline = time.time() + 3
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print("  nano: %s" % line)
        if line.startswith("READY"):
            break
    return ser


class ServoController:
    """Importable steering interface for the autonomy/ROS driver code.

    Wraps the serial link so callers think in steering, not bytes:
        steer(norm)  -> normalized -1 (full left) .. +1 (full right)
        set_angle(d) -> raw degrees (still clamped by the sketch)
        center()     -> straight ahead
        detach()     -> relax the servo (stop holding) when idle
    Use as a context manager so the port is always closed (and the servo
    detached) on exit, even if the caller crashes.
    """

    def __init__(self, port=None, baud=115200, open_now=True):
        self.port = port or find_port()
        if not self.port:
            raise RuntimeError("No Arduino serial port found (pass port=...).")
        self.baud = baud
        self.ser = None
        if open_now:
            self.open()

    def open(self):
        if self.ser is None:
            self.ser = connect(self.port, self.baud)
        return self

    def steer(self, norm):
        """norm in -1..+1; values outside are clamped (never past the stop)."""
        self.set_angle(norm_to_angle(norm))

    def set_angle(self, deg):
        # Clamp here too so a raw-degree caller can't exceed the safe range even
        # if the sketch hasn't been reflashed with the matching limits.
        deg = int(round(clamp(deg, LEFT_LIMIT, RIGHT_LIMIT)))
        send_quiet(self.ser, str(deg))

    def center(self):
        send_quiet(self.ser, "c")

    def detach(self):
        send_quiet(self.ser, "d")

    def close(self):
        if self.ser is not None:
            try:
                self.center()
                self.detach()
            except Exception:
                pass
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()


def send(ser, cmd):
    """Send one command line and print whatever the Nano replies."""
    ser.write((cmd + "\n").encode())
    ser.flush()
    time.sleep(0.05)
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            break
        print("  nano: %s" % line)


def send_quiet(ser, cmd):
    """Send a command without waiting for / printing the reply (smooth motion)."""
    ser.write((cmd + "\n").encode())
    ser.flush()


def glide(ser, a, b, step=2, dt=0.03):
    """Move gradually from angle a to b in small steps — low, gentle current."""
    a, b = int(a), int(b)
    rng = range(a, b + 1, step) if b >= a else range(a, b - 1, -step)
    for ang in rng:
        send_quiet(ser, str(ang))
        time.sleep(dt)
    time.sleep(0.1)
    ser.reset_input_buffer()   # drop the stream of ANGLE acks


def wiggle(ser):
    """Gentle oscillation around center so you can watch it move (no-load)."""
    print("Gentle glide: center -> right -> left -> center (small steps).")
    print("Watch the horn. If the Nano reboots (servo twitches then stops),")
    print("that's the USB brownout — needs the separate servo battery.\n")
    send(ser, str(CENTER))   # settle at center first (and show the ack)
    time.sleep(0.5)
    for _ in range(3):
        glide(ser, CENTER, RIGHT_LIMIT)
        glide(ser, RIGHT_LIMIT, LEFT_LIMIT)
        glide(ser, LEFT_LIMIT, CENTER)
    print("done — back at center.")


def interactive(ser):
    print("\nType an angle 0-180, 'c' center, 's' sweep, '?' status, 'q' quit.")
    while True:
        try:
            cmd = input("servo> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if cmd.lower() in ("q", "quit", "exit"):
            break
        if cmd:
            send(ser, cmd)


def quick_test(ser):
    """Center -> left -> center -> right -> center, with pauses to watch it."""
    for label, cmd in (("center", str(CENTER)), ("left", str(LEFT_LIMIT)),
                       ("center", str(CENTER)), ("right", str(RIGHT_LIMIT)),
                       ("center", str(CENTER))):
        print("-> %s (%s)" % (label, cmd))
        send(ser, cmd)
        time.sleep(0.8)


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("action", nargs="?", default=None,
                    help="angle 0-180 | center | sweep | test | steer N "
                         "(omit = interactive)")
    ap.add_argument("value", nargs="?", default=None,
                    help="for 'steer': normalized -1.0 (left) .. +1.0 (right)")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No Arduino serial port found. Plug in the Nano, or pass "
                 "--port /dev/ttyUSBx (and check: ls /dev/ttyUSB* /dev/ttyACM*).")

    print("Opening %s @ %d ..." % (port, args.baud))
    try:
        ser = connect(port, args.baud)
    except serial.SerialException as exc:
        sys.exit("Could not open %s: %s\nIs the Nano plugged in? Are you in the "
                 "'dialout' group? (groups | grep dialout)" % (port, exc))

    try:
        act = args.action
        if act is None:
            interactive(ser)
        elif act.lower() in ("c", "center"):
            send(ser, "c")
        elif act.lower() in ("s", "sweep"):
            send(ser, "s")
        elif act.lower() == "test":
            quick_test(ser)
        elif act.lower() in ("wiggle", "glide"):
            wiggle(ser)
        elif act.lower() == "steer":
            if args.value is None:
                sys.exit("steer needs a value: steer -1.0 (left) .. +1.0 (right)")
            try:
                angle = norm_to_angle(args.value)
            except ValueError:
                sys.exit("steer value must be a number in -1.0..+1.0")
            print("steer %s -> %d deg" % (args.value, angle))
            send(ser, str(angle))
        elif act.isdigit():
            send(ser, act)
        else:
            sys.exit("Unknown action %r (use 0-180, center, sweep, test, "
                     "steer N)." % act)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
