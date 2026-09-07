# Self-Driving Car — Project Build Log

**Platform:** NVIDIA Jetson Orin Nano (Super) on a Traxxas Slash 4×4 (4WD RC car)
**Goal:** Indoor autonomous driving — perceive surroundings with a camera (and later LiDAR), avoid obstacles class-agnostically, and drive both throttle and steering from the Jetson.
**Last updated:** 2026-06-26 (session: 60–115° steering limits flashed · web driving console in `gps_web.py` · `drive_test.py` · camera autonomy `autonav.py`)
**Working directory for code:** `Self Driving Car/Camera Control/` (steering driver in `Self Driving Car/servo/`)

> ⚡ **RESUME FAST:** jump to **§9 CURRENT STATUS & NEXT STEPS** at the bottom — it's written as the pick-up point.

---

## 1. System / Environment

| Item | Value |
|---|---|
| Board | Jetson Orin Nano Dev Kit (Super), power mode `MAXN_SUPER` (max) |
| JetPack | 6.2.1 (L4T r36.4.7) |
| CUDA | 12.6 |
| Python | 3.10.12 (system `/usr/bin/python3`) |
| OpenCV | 4.5.4 (system/JetPack build — **not** pip) |
| Display | X11 on `:1` |

**Golden rule:** never `pip install` generic `torch`, `opencv`, or CUDA packages — they break CUDA/cv2 on Jetson. All Python deps go in the **user site** (`~/.local`, via `pip install --user`) alongside the system cv2, with **numpy pinned `<2`** (numpy 2.x breaks cv2 4.5.4).

---

## 2. Camera

- USB webcam "icspring camera" → `/dev/video0` (uvcvideo). `video1` is a non-capture node.
- Only advertises **YUYV @ 640×480**, no MJPG → practical FPS ceiling **~18–22 fps**.
- FPS once dropped 15→9: cause was the camera's **auto-exposure** lengthening exposure in low light (NOT Jetson throttling — SoC stayed ~36 °C at max power). Fixed by pinning **manual exposure** (`auto_exposure=1`, `exposure_time_absolute=312`), baked into `live_preview.py`. Tunable later for motion blur on a moving car.

---

## 3. Software Stack Installed (the exact recipe)

All into `~/.local` (user site), numpy held at 1.26.4 (`<2`), system cv2 untouched.

1. **PyTorch (Jetson build, CUDA 12.6, nv24.08):**
   - `torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl` (807 MB)
   - `torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl`
   - (from the `ultralytics/assets` GitHub release v0.0.0)
2. **cuSPARSELt 0.6.3** (apt/deb, NVIDIA) — REQUIRED, torch won't import without `libcusparseLt.so.0`.
3. **numpy 1.26.4** — needed for the torch↔numpy bridge, still `<2` so cv2 is safe.
4. **Ultralytics 8.4.72** — installed `--no-deps` + manual deps (scipy, pandas, tqdm, psutil, py-cpuinfo, pyyaml, requests) to **avoid pulling `opencv-python`** (which would shadow the JetPack cv2).
5. **timm** — for MiDaS depth.
6. **onnx + onnxslim** — for TensorRT engine export (TensorRT 10.3 already in JetPack).
7. **pyserial 3.5** — VESC + microcontroller serial.
8. **arduino-cli 1.5.1** (`~/.local/bin`) + cores `arduino:avr` and `esp32:esp32@3.3.10` + libs `Servo`, `ESP32Servo`.

Verified: `torch.cuda.is_available() == True`, device "Orin" (compute 8.7), TensorRT 10.3.

---

## 4. Perception Pipeline (Camera Control/)

| File | Purpose |
|---|---|
| `live_preview.py` | Foundation: USB-camera live view, alignment overlay (crosshair/horizon/3×3 grid), FPS, q=quit s=save. Auto-detects the USB camera, applies manual exposure. |
| `calibrate_camera.py` + `make_checkerboard.py` | Camera intrinsic calibration (checkerboard 9×6 inner corners, 25 mm). **Not yet run** — needs the printed `checkerboard_9x6_25mm.pdf`. |
| `detect_image.py` | Stage-2 test: YOLO11n on a single image. |
| `live_detect.py` | Live YOLO11n (TensorRT) on the feed: boxes+labels+conf, keeps overlay+FPS, auto-uses `yolo11n.engine`. |
| `threaded_camera.py` | Low-latency capture: background thread, always-newest-frame, drops stale frames. Decouples inference from camera I/O. |
| `depth_nav.py` | **Obstacle avoidance:** MiDaS-small monocular depth → free-space columns → steering offset (−1..+1) + `blocked` flag. Class-agnostic. |
| `lane_detect.py` | Classic-CV lane/path detection. **NOT used for driving** (indoor = no lanes); kept for reference. |
| `live_drive.py` | Live view fusing depth-avoidance + YOLO labels + overlay. |
| `autodrive.py` | **Closed-loop autonomous driver** (see §6). |

**Performance (Orin Nano, yolo11n, 640):**
- YOLO PyTorch ~41 ms → **TensorRT FP16 ~29 ms** (engine `yolo11n.engine`, built with `half=True`).
- MiDaS-small depth ~45 ms (~22 fps depth-only).
- End-to-end: live detect ~17–18 fps; depth+YOLO ~13 fps; depth-only ~18 fps.
- **Bottleneck = the camera (~18 fps), not compute.** GPU ~50% idle → headroom for LiDAR/bigger models.

**Key insight:** YOLO alone is unsafe for "don't hit anything" (only 80 COCO classes — won't see walls/furniture/cables). Depth gives class-agnostic free space; YOLO adds semantics on top.

---

## 5. Actuation — Throttle (VESC) ✅ WORKING

- **Flipsky Mini FSESC 6.7 Pro** (VESC HW 6.x, firmware 5.2) drives the brushless motor.
- Connects to Jetson via **USB → `/dev/ttyACM0`**.
- **Motor detection done** in VESC Tool 7.00 (3S LiPo; Traxxas Velineon 3500 = **4 poles, sensorless**; R 10.4 mΩ, L 0.68 µH, flux 0.75 mWb). Direction was reversed → inverted in VESC Tool.
- Throttle control from Python via `vesc_driver.py` (`VESC` class: `set_duty`/`set_current`/`set_rpm`/`set_servo`/`get_values`, CRC16 packet protocol).
- **Tested:** 5% duty ramp spun all wheels smoothly (~2800 eRPM, ~4.6 A, 11.7 V, fault 0). ✅
- **`drive_test.py`** (new, 2026-06-26): gentle scripted bench test — forward 5% for 5 s → stop → reverse 5% → stop, streaming duty continuously with a ramp. `python3 drive_test.py check` = telemetry only (no motion). Re-confirmed forward+reverse, fault 0, Vin 11.3 V.
- ⚠️ **The VESC only enumerates while the MOTOR BATTERY is powered** (its logic is battery-fed, not USB). Battery off → `/dev/ttyACM0` disappears. If throttle "can't find the VESC", check the battery first.

**VESC servo output is NOT usable for steering** — researched: requires custom-compiled firmware (`SERVO_OUT_ENABLE`), which stock firmware lacks, and it's buggy (erratic pulses, FOC issues). Even Flipsky recommends an external microcontroller. → steering done separately.

---

## 6. Actuation — Steering (microcontroller) ✅ WORKING — **now the Arduino Nano (CH340)**

> **UPDATE 2026-06-26 — this is the current truth (the Uno/XIAO notes below are history):**
> Steering now runs on an **Arduino Nano clone (CH340)** on **`/dev/ttyUSB0`** (`by-id: usb-1a86_USB_Serial`).
> - Sketch: **`servo/arduino_servo/arduino_servo.ino`** (servo on **D9**). Protocol is **angle integers** (`"90"`, `c`=center, `d`=detach, `s`=sweep, `?`=status) — *not* the old float protocol.
> - Driver: **`servo/servo_control.py`** → `ServoController.steer(-1..+1)` maps to a **bench-tested ASYMMETRIC range: left = 60°, center = 90°, right = 115°** (the wheel connector binds outside this). Clamped in 3 places: the UI, `servo_control.py` (`LEFT_LIMIT`/`RIGHT_LIMIT`), and the firmware (`STEER_MIN=60`/`STEER_MAX=115`, **flashed this session** — confirmed: cmd 200→115, cmd 0→60).
> - Re-flash: `cd servo && bash flash.sh` (CH340 auto-reset is flaky — if it says "not in sync", just run it again).
> - The CH340 needs a kernel module the Jetson lacked → built from source in `servo/ch341-driver/`; `brltty` was removed (it grabs CH340 ports).
> - **`devices.py` does NOT find this Nano** (it looks for "Arduino"; CH340 reports `1a86`). So autonomy uses `ServoController` directly (see `autonav.py`), not `ArduinoSteering`.

*History (superseded):* In a normal RC car the **receiver** drives the steering servo; for autonomy the **Jetson replaces the receiver**. The VESC can't output a servo signal, so a microcontroller generates it.

- **Architecture:** Jetson → USB serial → microcontroller → servo PWM. Throttle stays Jetson → VESC.
- **Sketch protocol:** one float per line `-1.0`(left) .. `+1.0`(right); failsafe re-centers if no command for 1 s.
- **Worked on Arduino Uno** (`steering_arduino/steering_arduino.ino`, servo on D9): servo centered + moved L/R. ✅
- **Calibration:** `-1.0 = LEFT`, center = 1500 µs (straight), range **±450 µs** (full left 1050, full right 1950).
- **Now switching to Seeed XIAO ESP32S3** (smaller + WiFi/BLE for future wireless e-stop/telemetry). Sketch ready: `steering_esp32/steering_esp32.ino`, servo on **GPIO2 = pad "D1"**. Flash target `esp32:esp32:XIAO_ESP32S3`. **(In progress — see §9.)**

**Power lesson (important):** a microcontroller pin only provides the **signal**. The steering servo needs **external 5–6 V** (it draws >1 A under load). Do **NOT** power it from the Jetson 5 V (browns out the Jetson) or rely on the MCU. Use a **BEC/UBEC** for real driving (USB-5 V works for gentle bench testing only). Always **common ground** all boards.

**Stable device paths (use these — `ttyACM0/1` can swap!):** resolved in `devices.py`:
- VESC: `/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00`
- Steering MCU: by-id (Arduino = `usb-Arduino__...0043...`; XIAO = Espressif by-id — `devices.py` to be updated for the XIAO).

---

## 7. Closed-Loop Driving — use **`autonav.py`** (current); `autodrive.py` is STALE

> **UPDATE 2026-06-26:** The current autonomy script is **`Camera Control/autonav.py`** — wired to the verified hardware (Nano steering via `ServoController`, VESC throttle), **6% hard cap**. `autodrive.py` (below) is **stale**: it uses `ArduinoSteering`/`devices.py` which resolve to the old Uno on `/dev/ttyACM1` and will NOT find the current CH340 Nano.
>
> **`autonav.py` — camera free-space driving (the "is the front free → drive + steer" task):**
> - Perception: `ThreadedCamera` + `DepthNavigator` (MiDaS-small) → steer (−1..+1) + `blocked`.
> - Steering: `ServoController` (`../servo`, ttyUSB0, 60–115°). Throttle: `VESC.set_duty`, capped `MAX_DUTY=0.06`.
> - Added **EMA steer smoothing + center deadband** (`STEER_SMOOTH`, `STEER_DEADBAND=0.30`) so a broadly-open path goes straight instead of weaving toward the single most-open column.
> - Safety: throttle DISARMED at start; `a`=arm, `SPACE`=e-stop, `q`=quit (needs a display/NoMachine window). `blocked`→throttle 0. Exit/crash → stop motor + center wheels.
> - **VERIFIED:** `python3 autonav.py --dry` ran camera 640×480 + MiDaS at **~18 fps** producing live steer/blocked. **NOT yet floor-tested with motors** — next step is arm-on-a-stand.
> - Tuning if it weaves / stops too eagerly: `STEER_SMOOTH`/`STEER_DEADBAND`/`TURN_EASE` in `autonav.py`; `block_ratio`/`near_percentile`/`n_cols`/`drive_top` in `depth_nav.py`.

### 7a-legacy. `autodrive.py` (original, stand-tested with the OLD Uno)

Fuses perception + actuation with safety guardrails:
- **Throttle starts DISARMED**; steering always live. `a` = arm, `SPACE` = instant e-stop, `o` = toggle YOLO, `q` = quit.
- Hard **duty cap** (`MAX_DUTY=0.07`), smooth ramp, **eases off throttle in hard turns**, **`blocked` → throttle 0**.
- Failsafes: VESC auto-stops on comms loss (~1 s); MCU re-centers steering on comms loss (~1 s); on exit → stop + center.
- **Tested on the stand** (wheels up) with the Uno — steering tracked open space, throttle stopped on "blocked". (Was running when we paused to switch to the XIAO.)

---

## 7b. ROS 2 Integration (`~/sdc_ws`) ✅ scaffolded + builds

ROS 2 **Humble** is installed system-wide (`/opt/ros/humble`, `ROS_DISTRO=humble`,
`colcon`, plus `navigation2`/nav2 and `cv_bridge` already present). The standalone
`Camera Control/` scripts are NOT thrown away — a ROS package **wraps** them.

- **Workspace:** `~/sdc_ws` (kept space-free; the project dir has a space which ROS dislikes).
- **Package:** `sdc_drive` (ament_python). Nodes add `Camera Control/` to `sys.path`
  and import the REAL modules (`ThreadedCamera`, `DepthNavigator`, `VESC`,
  `ArduinoSteering`) — single source of truth, no fork. See `sdc_drive/legacy.py`.
- **Graph:** `camera_node` → `/camera/image_raw` (sensor_msgs/Image, cv_bridge) →
  `perception_node` (MiDaS depth) → `/perception/steer` (Float32 −1..+1) +
  `/perception/blocked` (Bool) → `driver_node` → VESC duty + steering MCU.
- **`driver_node`** re-implements the `autodrive.py` guardrails: throttle DISARMED
  by default, `max_duty=0.07` cap, smooth ramp, ease-off in turns, blocked→stop,
  perception **watchdog** (stale→stop), continuous commands (feeds VESC/MCU ~1 s
  failsafes), stop+center on exit. Arm via `/cmd/arm`, latched e-stop via `/cmd/estop`.
- **Verified:** `colcon build` clean; all 3 executables register; launch valid;
  `driver_node dry_run:=true` comes up disarmed with the right topics, no serial.
  (camera/perception not live-tested — they grab the camera + load MiDaS; run on the car.)

**Run:**
```bash
source /opt/ros/humble/setup.bash && source ~/sdc_ws/install/setup.bash
ros2 launch sdc_drive sdc.launch.py dry_run:=true   # perception graph, no serial
ros2 launch sdc_drive sdc.launch.py                 # full stack (stand first!), throttle disarmed
ros2 topic pub --once /cmd/arm std_msgs/Bool '{data: true}'   # arm throttle
```
Tuning: `~/sdc_ws/src/sdc_drive/config/params.yaml`. Details: that package's `README.md`.

**ROS-side next:** YOLO semantics topic · RPLIDAR driver (has ROS2 driver) · depth→costmap→**nav2**.

---

## 7c. GPS + Compass (Radiolink SE100) ✅ working

Radiolink **SE100** = u-blox **M8N GPS** + **IST8310** compass, on the 40-pin header.

- **GPS:** NMEA on `/dev/ttyTHS1` @ **38400** (pins 8/10, **TX/RX crossed**); gets a fix with sky view.
- **Compass:** **IST8310** on I2C **bus 7 @ `0x0e`** (WHO_AM_I=`0x10`), **sharing SDA/SCL (pins 3/5)** with the OLED `0x3c` — only one I2C pin pair exists, so the two devices are spliced onto the same lines.
- **Web dashboard:** `python3 GPS/gps_web.py` → `http://<jetson-ip>:8080` shows satellites connected + position + a **live rotating compass dial**.
- **Power-off lesson:** Jetson fully powered off while inserting compass wires = a **5V/GND short** (PMIC self-protect, worst under MAXN_SUPER); board undamaged. Power down before touching the header.
- **Caveats:** compass is **raw/uncalibrated** (needs 360° spin cal) and **must be mast-mounted away from the VESC/motors** or headings swing.

**Full details, wiring table, commands, scripts:** see **`GPS/GPS_COMPASS_LOG.md`**.

---

## 7d. Manual Web Driving Console (`GPS/gps_web.py`) ✅ NEW 2026-06-26

The GPS dashboard (`python3 GPS/gps_web.py` → `http://<jetson-ip>:8080`) was extended into a **manual driving console** so you can steer + drive from a browser/phone. Pure `http.server`, no framework. Updates pushed via `POST /servo` and `POST /throttle`; status merged into `/data`.

- **Manual steering card:** toggle to open the Nano link (lazy), then a slider you can **drag OR mouse-scroll** to set wheel angle; center button. Maps to `ServoController.steer` → 60–115°.
- **Throttle card (deadman):** backed by the VESC.
  - **■ EMERGENCY STOP** — big red, latches motor off until cleared.
  - **Lock / Unlock** — motor is dead until unlocked.
  - **+ / −** — set throttle level (default 5%, hard cap **20%**).
  - **Vertical lever** — *hold* up = forward, down = reverse, **release = stop** (springs back to center).
  - **Server-side deadman:** a background thread streams duty at ~20 Hz; if the browser stops sending keepalives (tab hidden, focus lost, network drop) throttle is cut within **0.5 s**.
- Both lazily open their port (steering=ttyUSB0 on enable, VESC=ttyACM0 on arm) and **release it when off** — so they don't permanently hold the ports.
- **Restart cleanly from a shell:** `cd GPS && setsid python3 gps_web.py >/tmp/gps_web.log 2>&1 </dev/null &` (plain `nohup ... & disown` inside a compound command got killed → exit 144).
- ⚠️ **Port conflict:** the dashboard and `autonav.py` both want ttyUSB0/ttyACM0 — only one at a time. Lock/disable in the UI (or stop the server) before running `autonav.py`, and vice-versa.

---

## 8. Jetson Mobile Power Plan (not yet built)

- **Separate 3S LiPo** (NOT the motor pack — motor spikes/sag would reset the Jetson) → **buck-BOOST set to 15 V, ≥100 W/≥6 A** → barrel jack **5.5×2.5 mm, CENTER-POSITIVE** → Jetson (accepts 9–19 V; ~25 W peak MAXN_SUPER, ~40 W system).
- Add **10 A inline fuse** + **LiPo low-voltage alarm** (~3.3 V/cell — RC LiPos have no BMS) + **common ground**.
- Runtime: 3S 5000 mAh ≈ 1.5–2 h. (For a DIY Li-ion pack instead: use **4S** + a **BMS** — a balance charger does NOT replace a BMS.)

---

## 9. CURRENT STATUS & NEXT STEPS (⭐ RESUME HERE ⭐)

### ✅ Done and verified
- Camera + perception (depth avoidance + YOLO/TensorRT) ✅
- **Steering = Arduino Nano (CH340) on `/dev/ttyUSB0`**, limits **60/90/115°** flashed & confirmed (§6) ✅
- **Throttle = VESC on `/dev/ttyACM0`**, `drive_test.py` forward/reverse confirmed (§5) ✅
- **Web driving console** in `gps_web.py`: scroll/drag steering + deadman throttle lever + E-STOP/lock (§7d) ✅
- **`autonav.py` camera autonomy** — perception verified (`--dry`, ~18 fps); throttle capped at 6% (§7) ✅
- GPS + compass (SE100) live + web dashboard ✅ (§7c) · ROS 2 scaffold builds ✅ (§7b)

### ⏸️ Where we stopped (state at power-off, 2026-06-26)
- Dashboard (`gps_web.py`) was **stopped** to free the serial ports.
- **VESC was OFFLINE** at shutdown — `/dev/ttyACM0` had disappeared (motor battery was off/unplugged). Steering Nano (`ttyUSB0`) was present.
- `autonav.py` is **written and perception-tested, but NOT yet floor-tested with motors.**

### ▶️ NEXT STEP: first powered run of `autonav.py` (do this next)
1. **Power up:** connect the **motor battery** to the VESC, and the **servo battery** (separate 5–6 V supply, common ground) for the steering. Plug both USB links into the Jetson.
2. **Confirm both devices are present:**
   ```bash
   ls /dev/serial/by-id/
   # expect:  usb-1a86_USB_Serial...  (steering Nano, →ttyUSB0)
   #          usb-STMicroelectronics_ChibiOS...  (VESC, →ttyACM0)
   ```
   If the VESC link is missing → its battery isn't on. If steering is missing → check the CH340/servo battery.
3. **Make sure nothing else holds the ports** — the web console grabs them when steering is enabled / throttle armed. Stop it: `pkill -f "python3 gps_web.py"` (or just don't open it).
4. **Stand test FIRST (wheels up), on the NoMachine desktop** (need a window for the keys):
   ```bash
   cd "Camera Control"
   python3 autonav.py --dry          # optional: perception only, no motors
   python3 autonav.py                 # throttle DISARMED; watch steering track open space
   # press 'a' to ARM throttle, SPACE = e-stop, 'q' = quit
   ```
   Confirm: steering turns toward open space; throttle stays 0 until armed; throttle drops to 0 when something blocks the center ("BLOCKED").
5. **Only then, on the floor:** clear space, finger on SPACE, arm briefly. Tune `STEER_DEADBAND`/`STEER_SMOOTH`/`block_ratio` if it weaves or stops too eagerly.

### 🔭 Later / backlog
- Build the Jetson battery power (§8) to go untethered.
- Add **RPLIDAR C3** (2D 360° DToF, 5 V USB) and fuse LiDAR ranging + camera semantics for robust avoidance.
- Tune exposure (shorter, for motion blur) and depth thresholds on the real driving surface.
- Run camera calibration (print the checkerboard) if metric geometry is needed.
- **Compass 360° calibration** + mast-mount the SE100 away from VESC/motors; bridge GPS fix + heading into ROS (`~/sdc_ws`).
- Wire the current Nano steering into the ROS `driver_node` (it still references the old `ArduinoSteering`).
- (Optional) Fix `devices.py` to also resolve the CH340 Nano by-id, so `autodrive.py`/ROS can find it.

---

## 10. Lessons / Gotchas

- **`pkill -f`** can match its own command line — use a bracket trick (`[v]esc_tool`) AND don't reference the target filename literally elsewhere in the same command.
- **VESC servo output** = custom firmware only + buggy → use an external MCU for steering.
- **Servo power** ≠ MCU job — needs an external BEC; never the Jetson 5 V.
- **numpy must stay `<2`** or system cv2 breaks; torch needs `cuSPARSELt`.
- **Engine files** (`yolo11n.engine`) are hardware/TensorRT-version specific — rebuild if JetPack changes.
- **Camera is the FPS bottleneck**, not compute — a faster camera is the biggest perception upgrade.

---

## 11. How to Run

```bash
cd "/home/wasif/Documents/Self Driving Car"

# --- ⭐ ONE-COMMAND AUTONOMY LAUNCHER (type this from anywhere) ---
drivecar            # full camera autonomy: forces DISPLAY=:0, auto-raises the
                    #   window in front of VS Code. Keys: a=arm SPACE=estop q=quit
drivecar --dry      # perception only, no motors
drivecar --max-duty 0.05   # gentler throttle cap
#   (alias in ~/.bashrc -> Self Driving Car/drive.sh)
#   WHY IT EXISTS: autonav's OpenCV window opens on whatever $DISPLAY the shell
#   inherited and BEHIND a maximized VS Code. The Jetson runs two X displays
#   (:0 = the NoMachine/GNOME desktop you actually see + where VS Code lives;
#   :1001 = a NoMachine virtual session). If the camera "doesn't show up", the
#   window is on the wrong display or hidden behind VS Code — drive.sh fixes both
#   (pins :0, then xdotool windowraise). Manual fallback: Alt-Tab to "AutoNav".

# --- Perception / autonomy (Camera Control/) ---
cd "Camera Control"
python3 live_preview.py        # camera + alignment overlay
python3 live_detect.py         # live YOLO11n (TensorRT)
python3 live_drive.py          # depth avoidance + YOLO labels (view only)
python3 autonav.py --dry       # CURRENT autonomy, perception only (no motors)
python3 autonav.py             # CURRENT autonomy, closed-loop (car on a STAND first!)
# (autodrive.py is STALE — wired to the old Uno on ttyACM1; use autonav.py)

# --- Steering bench control (servo/) ---
cd ../servo
python3 servo_control.py            # interactive: type angle 60..115, c=center, q
python3 servo_control.py steer -1   # full left ; steer 1 = full right
bash flash.sh                        # re-flash the Nano (run twice if "not in sync")

# --- Throttle bench test (Camera Control/) ---
cd "../Camera Control"
python3 drive_test.py check    # VESC telemetry only (no motion)
python3 drive_test.py          # fwd 5% 5s -> stop -> reverse 5% (wheels up!)

# --- Web driving console + GPS dashboard ---
cd ../GPS
setsid python3 gps_web.py >/tmp/gps_web.log 2>&1 </dev/null &   # -> http://<jetson-ip>:8080
pkill -f "python3 gps_web.py"  # stop it (frees ttyUSB0 + ttyACM0 for autonav.py)
```
