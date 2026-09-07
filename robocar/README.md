# robocar — clean rebuild

A fresh, organized rebuild of the self-driving car software. **No hardware
changes** — the wiring is fixed, so every device path/baud/limit is verified
once and lives in exactly one place (`config/hardware.py`). We add modules here
one at a time, each tested before the next.

The old code (`Camera Control/`, `servo/`, `GPS/`, `LiDAR/`, `~/sdc_ws`) stays
untouched as reference; we port the proven logic over cleanly as we go.

## Layout
```
robocar/
├── config/      device paths, bauds, limits, vehicle model   [1 file so far]
├── drivers/     thin, safe wrappers: VESC, steering, lidar, camera
├── perception/  camera -> free-space / obstacles
├── control/     arming, e-stop, safety, the driving loop
└── apps/        runnable entry points (bench tests, autonomy)
```

## Design rules
1. **One source of truth for hardware.** Nothing hardcodes `/dev/...` except
   `config/hardware.py`. Resolve every USB device by its `by-id` identity so a
   replug/added device can never point us at the wrong board.
2. **Safety first.** Throttle starts disarmed; every actuator has a failsafe
   stop/center on exit; only one process may own the VESC (`ttyACM0`) at a time.
3. **Add one brick, test it, then the next.** Each module is runnable/verifiable
   on its own before anything depends on it.

## Docs
- **docs/hardware_report.html** — full in-depth hardware bible: every component,
  the 40-pin GPIO wiring, USB/serial map, power architecture, per-subsystem
  deep-dives, the build journey, and gotchas. Open in a browser.

## Progress log
- [x] **config/hardware.py** — single source of truth for all devices.
      Self-test: `python3 robocar/config/hardware.py` (prints live device status;
      verified 2026-08-08 — all devices resolve by-id correctly).
- [ ] drivers/steering.py — steering wrapper (by-id Nano, 60/90/115°)
- [ ] drivers/vesc.py — throttle + telemetry wrapper
- [ ] drivers/camera.py — threaded latest-frame camera
- [ ] drivers/lidar.py — RPLIDAR C1 scans
- [ ] perception/… — depth free-space
- [ ] control/… — arming + safety + loop
- [ ] apps/… — bench tests, then autonomy

## Hardware quick reference (verified 2026-08-08)
| Device | by-id resolves to | Baud / addr |
|---|---|---|
| VESC (throttle) | `/dev/ttyACM0` | 115200 |
| Steering Nano (CH340) | `/dev/ttyUSB1` | 115200 |
| RPLIDAR C1 (CP2102N) | `/dev/ttyUSB0` | 460800 |
| GPS SE100 | `/dev/ttyTHS1` | 38400 |
| Compass IST8310 | i2c-7 | 0x0e |
| OLED | i2c-7 | 0x3c |
| Camera ×2 (icSpring) | /dev/video0–3 | YUYV 640×480 |

Steering: 60°=left / 90°=center / 115°=right (asymmetric, bench-tested).
Vehicle: meters_per_tach=0.003424 (calibrated); wheelbase/max-steer still TODO.
