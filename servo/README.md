# Steering Servo — Arduino Nano bridge

The VESC firmware wouldn't drive the steering servo, so an **Arduino Nano** sits
between the Jetson and the servo and generates the 50 Hz pulse:

```
Jetson  --USB serial-->  Arduino Nano  --D9 (signal)-->  Steering servo
```

## Wiring

| Servo wire | Goes to |
|---|---|
| Signal (orange/white) | Arduino **D9** |
| `+` (red) | **5 V** — see power note below |
| `-` (brown/black) | **GND** (shared with the Arduino GND) |

Arduino Nano → Jetson over USB. It enumerates as **`/dev/ttyUSB0`** (CH340
USB-serial chip), which is *different* from the VESC on `/dev/ttyACM0`.

### Battery power for the servo (do this before driving)

A no-load bench test ran with the servo on the Nano's 5V, but under any load the
servo browns out the Nano (it resets / the servo just shakes). Move servo power
to a separate **5–6 V battery/BEC**, keeping a **common ground**:

```
Servo signal (orange) --> Nano D9        (unchanged)
Servo +      (red)     --> Battery/BEC +  (5–6 V)   <-- NOT the Nano 5V
Servo -      (brown)   --> Battery/BEC -
Battery/BEC -          --> Nano GND        <-- common ground, ESSENTIAL
```

The only change from the bench setup is moving servo `+` off the Nano 5V onto the
battery, and tying the battery `−` to a Nano GND. Signal stays on D9.

## Jetson CH340 driver setup (one-time, already done)

This Jetson kernel (`5.15.x-tegra`) **does not ship `ch341.ko`**, and the
generic `usbserial` driver here doesn't support manual IDs — so the CH340 Nano
got no `/dev/ttyUSB0`. Also, **`brltty` hijacks CH340 ports** on Ubuntu 22.04.
Both are fixed:

- `brltty` removed (`sudo apt-get remove -y brltty`).
- `ch341` driver built from upstream 5.15 source and installed; it now
  auto-loads on plug. Source + rebuild script live in `ch341-driver/`.

If `/dev/ttyUSB0` ever stops appearing (e.g. after a **kernel update**), rebuild:

```bash
cd "servo/ch341-driver"
./install_ch341.sh        # builds, installs, depmod, modprobe
```

> ⚠️ **Power note.** A steering servo can pull **>1 A** when it hits a stop or
> fights the wheels. Running that through the Arduino's 5V (fed from the Jetson
> USB) can brown out the Nano or the Jetson port and cause resets/glitches.
> A quick *free-air, no-load* test off USB power is usually fine, but for the
> car put the servo on a **separate 5–6 V BEC** and just share GND between the
> BEC, the servo, and the Arduino.

## Files

| File | What it is |
|---|---|
| `arduino_servo/arduino_servo.ino` | Nano firmware. Reads angle commands over serial, drives the servo on D9 with the `Servo` library. |
| `servo_control.py` | Jetson-side tester. Sends angles and prints the Nano's replies. |

## 1. Flash the Arduino

Use the Arduino IDE (or `arduino-cli`):
1. Open `arduino_servo/arduino_servo.ino`.
2. Board: **Arduino Nano**. If upload fails, switch Processor to
   **ATmega328P (Old Bootloader)** — most Nano clones need this.
3. Port: the Nano's `/dev/ttyUSB0`. Upload.

Sanity check in the IDE Serial Monitor (115200 baud, Newline): you should see
`READY ...`. Type `s` → the servo should sweep. Type `90` → it centers.

## 2. Drive it from the Jetson

```bash
cd "servo"
python3 servo_control.py            # interactive: type 0..180, c, s, q
python3 servo_control.py test       # center → left → center → right → center
python3 servo_control.py sweep      # one full sweep
python3 servo_control.py 120        # set a single angle and exit
# force a port if auto-detect guesses wrong:
python3 servo_control.py test --port /dev/ttyUSB0
```

If it says it can't open the port: confirm the Nano is plugged in
(`ls /dev/ttyUSB* /dev/ttyACM*`) and that you're in the `dialout` group
(`groups | grep dialout`; if missing: `sudo usermod -aG dialout $USER` then
log out/in).

## Serial protocol (so you can talk to the Nano from anything)

115200 baud, newline-terminated, ASCII:

| Send | Effect | Nano replies |
|---|---|---|
| `0`..`180` | set angle (degrees) | `ANGLE <n>` |
| `c` | center (90°) | `ANGLE 90` |
| `s` | sweep min→max→center once | `SWEEP done` |
| `?` | status | `STATUS angle=<n>` |

## Status

- ✅ Driver + port working (`/dev/ttyUSB0`), sketch flashes, Jetson↔Nano serial OK.
- ✅ Servo **receives the signal** — it twitches/shakes to commands.
- ⏳ **Needs the servo battery** to actually move/hold position (USB power browns
  out the Nano). Add the BEC per the wiring above, then re-test.

## Next steps

- After the battery is in: `python3 servo_control.py wiggle` should sweep
  smoothly. Then `python3 servo_control.py` and find the real **steering limits**
  (left-lock / center / right-lock angles) — they won't be 0/90/180 on a car
  linkage. Note them; the perception steering command maps onto that range.
- Then bridge it: camera/perception steer value → angle → `servo_control` →
  Nano, alongside the VESC throttle. (See the VESC notes for the throttle half.)
