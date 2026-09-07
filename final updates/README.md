# final updates — main working folder

**This is now the primary workspace.** From here on, all new code, tests, configs,
and docs for the self-driving car go in here. The older folders
(`Camera Control/`, `servo/`, `GPS/`, `LiDAR/`, `Camera Lab/`, `robocar/`,
`~/sdc_ws`) stay as **reference** — proven logic is ported *into* here cleanly, not
edited in place.

> **No hardware/wiring changes — ever.** Everything is soldered and set up. The
> wiring is fixed ground truth (see the hardware references below); we only write
> and improve software against it.

## Verified hardware quick-reference (2026-08-08)
| Device | stable identity → now | baud / addr |
|---|---|---|
| VESC (throttle) | `usb-STMicroelectronics_ChibiOS…304-if00` → ttyACM0 | 115200 |
| Steering Nano (CH340) | `usb-1a86_USB_Serial-if00-port0` → ttyUSB1 | 115200 (servo D9, 60/90/115°) |
| RPLIDAR C1 (CP2102N) | `usb-Silicon_Labs_CP2102N…cad4bd81…-if00-port0` → ttyUSB0 | 460800 |
| GPS SE100 (M8N) | `/dev/ttyTHS1` (pins 8/10 crossed) | 38400 |
| Compass IST8310 | i2c-7 | 0x0e |
| OLED SSD1306 | i2c-7 | 0x3c |
| **Camera FRONT** | by-path `…usb-0:2.1:1.0-video-index0` | YUYV 640×480, upright |
| **Camera REAR** | by-path `…usb-0:2.3:1.0-video-index0` | YUYV 640×480, **rotate 180°** |

Both cameras confirmed working 2026-08-08. Cameras are identical models, so they are
pinned by **physical USB port path** (front = port 2.1, rear = port 2.3), never by
`/dev/videoN` (those numbers can swap on replug). Rear is mounted upside down →
always rotate 180°.

## Reference docs (read these; they guide the work)
- `docs/camera_perception_report.md` — camera & perception plan (front/rear jobs,
  model shortlist, depth→costmap, nav2 vs reflex, calibration, roadmap).
- `../robocar/docs/MIGRATION_PLAN.md` — clean-rebuild plan (wiring ground truth,
  env recipe, what-ports-where, brick-by-brick build order, where SLAM fits).
- `../robocar/docs/hardware_report.html` — full hardware bible.
- `../robocar/config/hardware.py` — the existing single-source-of-truth device config.

## Planned layout (folders created as we build)
```
final updates/
├── config/       device paths, bauds, limits, camera by-path + rotation
├── drivers/      thin safe wrappers: vesc, steering, camera(×2), lidar, compass, gps
├── perception/   depth→free-space/costmap, detection+track+distance, drivable-area
├── control/      arming, e-stop/reflex, odometry, driving loop
├── apps/         runnable entry points (bench tests, live viewers, autonomy)
└── docs/         references + notes
```

> Note on the folder name: this folder has a space in its name (as requested).
> Python packages can't have spaces in their import names, so any importable
> package folders created inside will use space-free names (e.g. `config/`,
> `drivers/`) — the top-level "final updates" folder stays as-is.

## Status
- [x] Cameras probed + confirmed (both work); front/rear pinned by USB port.
- [x] Rear-camera 180° rotation established.
- [ ] Everything else — built here, one tested brick at a time.
