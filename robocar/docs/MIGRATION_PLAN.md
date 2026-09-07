# Self-Driving Car — Migration Plan (clean rebuild)

**Author target:** rebuild the working car onto a clean, safe, single-source-of-truth
codebase (`robocar/`) **without touching any hardware or wiring** — everything is
soldered and set up, so the wiring below is fixed ground truth.

**Platform:** Traxxas Slash 4×4 · NVIDIA Jetson Orin Nano (Super) · JetPack 6.2.1
(L4T r36.4.7) · CUDA 12.6 · Python 3.10 · OpenCV 4.5.4 (system) · ROS 2 Humble installed.

**Decision (this migration):** Build the **`robocar/` plain-Python core first**
(drivers → perception → control → apps), reproducing the proven `autonav.py`
autonomy on clean code. **Then add a ROS 2 layer** for **SLAM (slam_toolbox, 2D
LiDAR) + navigation (nav2)**, which is where ROS earns its weight. The Python core
stays the single source of truth; ROS nodes *wrap* it, never fork it.

> Verified live 2026-08-08: all USB devices resolve by-id, `config/hardware.py`
> self-test passes. This plan builds forward from that point.

---

## 0. Ground truth — wiring & hardware (DO NOT change; migrate against this)

### 0.1 USB / serial devices (resolved by stable `by-id`, never by ttyN)

| Device | by-id identity | Now maps to | Baud | Protocol / notes |
|---|---|---|---|---|
| **VESC** Flipsky Mini FSESC 6.7 Pro (throttle) | `usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00` | `/dev/ttyACM0` | 115200 | Native VESC packet (CRC16). **Only enumerates when the MOTOR BATTERY is on** — logic is battery-fed, not USB. No battery ⇒ no port. |
| **Steering** Arduino Nano clone (CH340 / `1a86`) | `usb-1a86_USB_Serial-if00-port0` | `/dev/ttyUSB1` | 115200 | Servo signal on **D9**. Integer-degree ASCII protocol (`"90"`, `c`=center, `d`=detach, `s`=sweep, `?`=status). Needs custom-built `ch341.ko`. |
| **RPLIDAR C1** (Silicon Labs CP2102N) | `usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_cad4bd81365aee11899081dc8ffcc75d-if00-port0` | `/dev/ttyUSB0` | 460800 | 2D 360° DToF. Model 0x41, fw 1.01. **Set DTR low on open** so the motor isn't held in reset. |
| **Camera** ×2 icSpring USB | (opened by index) | `/dev/video0–3` | — | YUYV **640×480 only** (no MJPG) → ~18–22 fps ceiling. Manual exposure pinned (`exposure_time_absolute=312` ≈ 31 ms). Camera is the FPS bottleneck. |

> **Why by-id:** `ttyUSB0/1`, `ttyACM0` are assigned in enumeration order and SHIFT
> when any device is added/removed/replugged. Adding the RPLIDAR already pushed the
> steering Nano from `ttyUSB0`→`ttyUSB1`. Grabbing "the first ttyUSB*" would talk to
> the LiDAR instead of steering — dangerous. Always resolve by-id, fall back to raw.

### 0.2 40-pin header — GPS UART + shared I²C

| Signal | Jetson 40-pin | Detail |
|---|---|---|
| GPS **TX → Jetson** | **Pin 10** (UART1 RX) | **crossed** |
| GPS **RX ← Jetson** | **Pin 8** (UART1 TX) | **crossed** → `/dev/ttyTHS1` @ **38400** (not u-blox 9600 default) |
| I²C **SDA** | **Pin 3** | bus `/dev/i2c-7` |
| I²C **SCL** | **Pin 5** | bus `/dev/i2c-7` |
| Compass **IST8310** | i2c-7 @ **0x0e** | WHO_AM_I(reg 0x00)=0x10. Spliced onto the same SDA/SCL as the OLED (only one I²C pair exists). |
| OLED **SSD1306** | i2c-7 @ **0x3c** | VCC on **3.3 V (pin 1)**, *not* 5 V. |

GPS module = **Radiolink SE100** = u-blox M8N GPS + IST8310 compass on a 6-pin
Pixhawk-style connector (`VCC, GPS-TX, GPS-RX, SCL, SDA, GND`).

### 0.3 Steering geometry (bench-tested, ASYMMETRIC — linkage swings further left)

```
left = 60°   center = 90°   right = 115°
norm −1.0 ───────── 0.0 ───────── +1.0
```
Clamped in **three** places, keep in sync: firmware `arduino_servo.ino`
(`STEER_MIN 60`/`STEER_MAX 115`), `servo_control.py` (`LEFT_LIMIT`/`RIGHT_LIMIT`),
and the UI. The wheel connector *binds* outside this range — never command past it.

### 0.4 Vehicle model / calibration

- **`meters_per_tach = 0.003424`** — CALIBRATED 2026-07-23 (pushed 6.8 m → Δtach 1986).
  This is the key to LiDAR-SLAM odometry.
- Throttle safety caps: `max_duty = 0.10` hard ceiling, `default_duty = 0.05`.
- **TODO (measure on the real chassis):** `wheelbase_m` (≈0.25 provisional),
  `max_steer_angle_rad` (≈0.45 provisional) — needed for Ackermann odometry math.

### 0.5 Hard-won power / safety rules (carry into every new module)

- **Servo needs its own 5–6 V BEC**, common ground with the Nano. NEVER the Jetson
  5 V (>1 A stall current browns out the Jetson/Nano).
- **Compass corrupts near the motor** — mast-mount the SE100 away from VESC/motor/
  power wiring, and its cal is throttle-dependent (re-test with motor running).
- **Power down before touching the 40-pin header** — a live 5V/GND short once
  powered the whole board off (PMIC self-protect; worst under MAXN_SUPER).
- **Jetson mobile power (not yet built):** separate 3S LiPo (NOT the motor pack) →
  buck-boost 15 V ≥100 W → center-positive 5.5×2.5 barrel jack; 10 A fuse + LVC alarm.
- **One owner per serial port:** the web console and autonomy both want ttyUSB1 +
  ttyACM0. Only one process at a time — enforce in code.

---

## 1. Software environment — the golden rules (reproduce, don't fight)

These are non-negotiable on this Jetson; the new code assumes them.

1. **Never `pip install` generic `torch`, `opencv`, or CUDA packages** — they break
   the JetPack CUDA/cv2. All Python deps go to the **user site** (`pip install --user`).
2. **numpy pinned `<2` (1.26.4)** — numpy 2.x breaks system cv2 4.5.4.
3. **PyTorch = the Jetson wheel** (`torch-2.5.0a0+…nv24.08 cp310 aarch64`) + torchvision.
   Requires **cuSPARSELt 0.6.3** (apt) or torch won't import.
4. **Ultralytics** installed `--no-deps` (+ manual deps) so it can't pull
   `opencv-python` and shadow the JetPack cv2.
5. **transformers ≥ 4.45** for Depth-Anything V2 metric head (4.44 outputs zeros).
6. TensorRT 10.3 (JetPack). **Engine files are hardware/TensorRT-version specific** —
   rebuild `yolo11n.engine` if JetPack changes.
7. Groups: user in `dialout` (serial) + `i2c`. `brltty` removed (it hijacks CH340).
8. **CH340 driver** built from source (`servo/ch341-driver/install_ch341.sh`) — rerun
   after any kernel update or `/dev/ttyUSB*` for the Nano disappears.

**Action:** freeze this as `robocar/docs/ENVIRONMENT.md` + a `requirements-user.txt`
(pinned, `--user`) and a one-shot `scripts/setup_env.sh` that is *safe to re-run*.

---

## 2. Target layout (`robocar/`)

```
robocar/
├── config/
│   ├── hardware.py        ✅ DONE — single source of truth (ports, bauds, geom, model)
│   └── params.py          [new] tunables (control gains, perception thresholds)
├── drivers/               thin, SAFE wrappers — each runnable/testable standalone
│   ├── vesc.py            throttle + telemetry + tach odometry
│   ├── steering.py        Nano servo, norm −1..+1 → 60/90/115°
│   ├── camera.py          threaded latest-frame USB capture
│   ├── lidar.py           RPLIDAR C1 scans (self-contained)
│   ├── compass.py         IST8310 heading (shared i2c-7)
│   └── gps.py             SE100 NMEA parse
├── perception/
│   ├── depth_metric.py    Depth-Anything V2 metric → free-space (primary)
│   ├── depth_relative.py  MiDaS relative (legacy fallback)
│   └── yolo.py            YOLO11n TensorRT semantics (optional overlay)
├── control/
│   ├── safety.py          arming, e-stop, watchdog, "one owner" port lock
│   ├── odometry.py        VESC tach + heading → (x, z, yaw) pose track
│   └── loop.py            the drive loop (steer smoothing, deadband, turn-ease,
│                          distance-proportional speed, dead-end recovery)
├── apps/
│   ├── check_hardware.py  status report (extends hardware.py self-test)
│   ├── bench_steering.py  / bench_throttle.py  — safe wheels-up tests
│   ├── autonav.py         reproduce the proven camera autonomy, clean
│   └── teleop_web.py      the web driving console (deadman throttle + steering)
└── docs/
    ├── hardware_report.html  ✅ DONE — full hardware bible
    ├── MIGRATION_PLAN.md     (this file)
    └── ENVIRONMENT.md        [new] the env recipe above
```

**Design rules:** (1) nothing hardcodes `/dev/...` except `config/hardware.py`;
(2) throttle starts disarmed, every actuator fails safe (stop/center) on exit;
(3) add one brick, test it, then the next.

---

## 3. What to port from where (proven logic → new home)

| New file | Port from (reference, keep untouched) | What carries over |
|---|---|---|
| `drivers/vesc.py` | `Camera Control/vesc_driver.py` | `VESC` class: CRC16 packet framing, `set_duty/current/rpm`, `get_values` (adds tach for odometry), context-manager stop-on-exit. |
| `drivers/steering.py` | `servo/servo_control.py` + `servo/arduino_servo/arduino_servo.ino` | `ServoController.steer(−1..+1)` → asymmetric 60/90/115° mapping; open→READY handshake; center+detach on close. |
| `drivers/camera.py` | `Camera Control/threaded_camera.py` (+ `live_preview.py` detect/exposure) | Threaded latest-frame-wins capture, seq-id `read(wait=True)`, manual exposure. Decouple from `live_preview` import. |
| `drivers/lidar.py` | `LiDAR/rplidar_c1.py` | Self-contained C1 driver: connect/reset, get_info/health, `iter_scans()` → (quality, angle°, dist_mm). DTR-low on open. |
| `drivers/compass.py` | `GPS/compass_read.py` + `compass_cal.py` | IST8310 read sequence, cal-offset loading, heading. |
| `drivers/gps.py` | `GPS/gps_read.py` | NMEA parse → lat/lon/sats/fix. |
| `perception/depth_metric.py` | `Camera Control/metric_nav.py` + `Camera Lab/depth_engine.py` | Depth-Anything V2 metric; per-column reach; openness-weighted steering centroid; blocked+hysteresis. **Primary — sees flat walls.** |
| `perception/depth_relative.py` | `Camera Control/depth_nav.py` | MiDaS relative depth free-space (legacy fallback; can't see flat walls). |
| `perception/yolo.py` | `Camera Control/live_detect.py` | YOLO11n TensorRT boxes/labels (semantics overlay only; not the safety layer). |
| `control/loop.py` + `control/safety.py` | `Camera Control/autonav.py` | The whole closed loop: EMA steer smoothing, center deadband, turn-ease, distance-proportional speed, **dead-end recovery** (3-point-turn escape), disarmed-start, e-stop, watchdog, stop+center on exit. |
| `apps/teleop_web.py` | `GPS/gps_web.py` | Web console: deadman throttle lever (server-side 0.5 s cutoff), scroll/drag steering, E-STOP/lock, lazy port open/release. Trim GPS/compass/sys-stats into their own dashboard module. |
| `apps/check_hardware.py` | `robocar/config/hardware.py` `status_report()` | Extend with i2cdetect + WHO_AM_I + camera-open probes. |

**Left as reference / lab (do NOT block the migration on these):**
`Camera Lab/` (mapper.py, live_map.py visual SLAM experiments), `oled_stats/`
(already a clean standalone systemd service — leave it running as-is), the ROS 2
`~/sdc_ws` scaffold (superseded by the ROS phase below), `autodrive.py`,
`devices.py` (both stale).

---

## 4. Build & test order (one brick at a time — each verifiable before the next)

**Phase A — foundation & safety**
1. `config/params.py` + `docs/ENVIRONMENT.md` + `scripts/setup_env.sh`.
2. `apps/check_hardware.py` — confirm all devices resolve, i2c WHO_AM_I, camera opens.
3. `control/safety.py` — arming/e-stop/watchdog/port-lock primitives (unit-testable).

**Phase B — drivers (each with a `__main__` self-test)**
4. `drivers/steering.py` → `apps/bench_steering.py` (wheels-up sweep, limits).
5. `drivers/vesc.py` → `apps/bench_throttle.py` (`check` = telemetry only; fwd/rev ramp, wheels up).
6. `drivers/camera.py` → FPS self-test.
7. `drivers/lidar.py` → print health + one scan; `scan_plot.png`.
8. `drivers/compass.py`, `drivers/gps.py` → live heading / NMEA fix.

**Phase C — perception**
9. `perception/depth_metric.py` → dry FPS + steer/blocked on live camera (target ~12 fps).
10. (optional) `perception/yolo.py`, `perception/depth_relative.py`.

**Phase D — autonomy (reproduce proven behavior, clean)**
11. `control/loop.py` + `apps/autonav.py` → `--dry` (perception only), then wheels-up
    arm test, then floor test. Must match today's `autonav.py` behavior.
12. `apps/teleop_web.py` → web console parity, deadman verified.

**Phase E — odometry (the bridge to SLAM)**
13. `control/odometry.py` → fuse VESC tach (`meters_per_tach`) + compass/GPS heading
    into an `(x, z, yaw)` pose track. Validate: push the car a known distance, check
    the track. **This is the prerequisite for LiDAR SLAM.**

**Phase F — SLAM + navigation (ROS 2)**  ← the SLAM you asked about
14. RPLIDAR C1 ROS 2 driver publishing `/scan` (`sensor_msgs/LaserScan`).
15. Bridge nodes: `robocar` core → ROS 2 — publish `/odom` + TF
    (`odom→base_link`, `base_link→laser`), subscribe cmd → VESC/steering.
16. **`slam_toolbox`** (Humble, already the standard for 2D LiDAR) — online async
    mapping from `/scan` + `/odom`. Produces a live occupancy map + corrected pose.
17. **`nav2`** (already installed) — costmap from the map + LiDAR, goal-directed
    driving with an Ackermann-aware controller. Camera depth stays as a
    complementary short-range obstacle layer.

> **SLAM notes for this hardware:** the RPLIDAR C1 is 2D 360°, which is exactly
> `slam_toolbox`'s sweet spot. It needs a reasonable `/odom` (Phase E) + a static
> TF for the laser mount. Your in-house `LiDAR/slam_test.py`/`slam_live.py` (scan-
> matching) are good for learning and a no-ROS fallback, but `slam_toolbox` is far
> more robust (loop closure, serialization) and pairs directly with `nav2`. The
> `Camera Lab/` visual SLAM stays experimental — it drifts and needs real poses.

---

## 5. Open calibration / TODO (independent of code — schedule alongside)

- **Camera intrinsics:** print `Camera Control/checkerboard_9x6_25mm.pdf`, run
  `calibrate_camera.py` → real `fx/fy/cx/cy` (depth cloud + any metric geometry).
- **Compass 360° spin cal** in the final mounted position, motor OFF then re-test
  with motor ON (throttle-dependent interference).
- **Chassis measurements:** `wheelbase_m`, `max_steer_angle_rad` (Ackermann/odometry).
- **Exposure tuning** for motion blur on the moving car (shorter than 31 ms + gain).
- **LiDAR mount TF:** measure the laser position/orientation vs `base_link`.

---

## 6. First concrete steps once you say go

1. Scaffold Phase A files (`params.py`, `ENVIRONMENT.md`, `setup_env.sh`,
   `check_hardware.py`, `safety.py`).
2. Port `drivers/steering.py` + `drivers/vesc.py` with wheels-up bench apps.
3. Bring up `drivers/lidar.py` and confirm a clean scan (SLAM input path).

Each step is a small, reviewable change with its own test — nothing depends on a
brick that hasn't been proven. Hardware and wiring stay exactly as they are.
