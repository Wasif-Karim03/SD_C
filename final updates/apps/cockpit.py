#!/usr/bin/env python3
"""
apps/cockpit.py — RoboCar MISSION CONTROL: one web app to run the whole car.

A single, memory-aware control center for the Jetson Orin Nano (8 GB). Instead of
running every heavy pipeline at once (which OOM-crashed the old cockpit), it uses
MODES that share one light always-on core (LiDAR + telemetry + steering + E-STOP):

  DRIVE       manual driving; live front camera + top-down LiDAR radar + 3D view.
  MAP         drive slowly while 2D LiDAR SLAM builds an indoor map; SAVE MAP.
  NAVIGATE    load the saved map, localize, click a goal, GO -> auto-drive the
              route (pure-pursuit) with live LiDAR obstacle stopping.
  PERCEPTION  YOLO object detection on the front (and optional rear) camera —
              "what the car sees". The detector loads only in this mode and is
              freed on the way out, so RAM stays under control.

Owns the hardware, so run it INSTEAD of mapper_web / navigate_web / control_center:

  cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
  python3 cockpit.py
  open http://<jetson-ip>:8080

E-STOP latches the motor off (top-right, always). Ctrl-C to quit.
"""
import os
import sys
import gc
import json
import math
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                             # noqa: E402
from drivers.lidar import ThreadedLidar                   # noqa: E402
from drivers.camera import Camera                          # noqa: E402
from perception.slam import LidarSLAM, RES, SIZE, ORIGIN   # noqa: E402
from perception import planner as P                        # noqa: E402
from perception.lidar_nav import LidarNavigator           # noqa: E402
try:
    from recording.recorder import Recorder                 # noqa: E402
except Exception as _rec_err:                               # noqa: BLE001
    Recorder = None
    print("  [rec] session recording unavailable:", _rec_err)
try:
    from drivers.gps_stream import ThreadedGPS               # noqa: E402
except Exception as _gps_err:                               # noqa: BLE001
    ThreadedGPS = None
    print("  [gps] GNSS stream unavailable:", _gps_err)

HTTP_PORT = 8080
WEB_DIR = os.path.join(HERE, "web")     # static front end, served at /web/

# Optional shared secret on COMMANDS. Unset by default, so nothing changes on
# a bench. Set it before driving anywhere with other people on the network:
#
#     ROBOCAR_TOKEN=somethinglong ./run_cockpit.sh
#     open http://<host>:8080/?k=somethinglong
#
# It gates POST only. Telemetry stays readable, because a colleague watching
# the numbers is harmless and being able to see what the car is doing is a
# safety property in itself. What it stops is anyone on the LAN being able to
# arm the motor of a vehicle they are not standing next to. This is a plain
# shared secret over plain HTTP: it is a lock on a door, not a security
# system, and it is worth exactly that much.
CMD_TOKEN = os.environ.get("ROBOCAR_TOKEN", "")
OUT = 500                       # map render size (px)
MAP_PATH = os.path.join(ROOT, "maps", "room.npy")
MAP_DIR = os.path.join(ROOT, "maps")
MAXD = config.MAX_DUTY
RAMP = 0.01
DEADMAN_S = 0.5
RADAR_SCALE = 20.0              # px per metre in the DRIVE radar view

# pure-pursuit (auto-drive)
LOOKAHEAD_M = 0.55
GOAL_TOL_M = 0.25
STEER_GAIN = 1.8
FOLLOW_STEER_SIGN = config.STEER_SIGN   # ONE knob for both followers — see config.py
FOLLOW_FORWARD_DEG = config.LIDAR_FORWARD_DEG
SCAN_MAX_AGE_S = 0.6            # auto-drive refuses to move on a scan older than this
REPLAN_EVERY_S = 1.0
REACT_M = 1.3
BLOCK_GIVEUP_S = 6.0
# Derived from the MEASURED breakaway rather than picked. Both of these used
# to be 0.07 and 0.09 -- below config.MIN_MOVE_DUTY as measured on 2026-09-09
# (0.14), which means autonomous follow has never been able to move this car.
# Hitting GO commanded a duty the vehicle cannot translate at, and the car sat
# there while the planner, the localizer and the pure-pursuit loop all worked
# perfectly.
#
# Note how little room there is: breakaway 0.14, MAX_DUTY 0.20. The entire
# speed-control authority of this vehicle is 0.06 of duty. That is a real
# constraint on what any controller here can do, learned or otherwise.
DUTY_INDOOR = round(config.MIN_MOVE_DUTY + 0.015, 3)
DUTY_OUTDOOR = round(config.MIN_MOVE_DUTY + 0.035, 3)

STATE = {}
S_LOCK = threading.Lock()
CTRL = {"armed": False, "estop": False, "throttle": 0.0, "steer": 0.0, "last_cmd": 0.0}
C_LOCK = threading.Lock()
GOAL = {"cell": None}
PATH = {"cells": None, "world": None}
FOLLOW = {"on": False, "arrived": False, "note": ""}

BLANK = None                    # placeholder jpeg

# Pack chemistry. The alarm limits are PER CELL — 3.5 V/cell is "land now",
# 3.3 V is where you start damaging a LiPo — so the pack limits depend on the
# cell count, which the cockpit had hardcoded for a 4S. On a 3S that turned a
# perfectly healthy 11.7 V into a screaming alarm.
#
# The count is inferred from the first sane reading rather than configured,
# because a wrong constant here is worse than no constant: it either cries
# wolf or stays silent while the pack is being ruined. Set BATTERY_CELLS in
# config.py to override.
V_CELL_ALARM = 3.30
V_CELL_WARN = 3.50
V_CELL_FULL = 4.25
BATT = {"cells": getattr(config, "BATTERY_CELLS", None), "inferred": False}


def _infer_cells(v_in):
    """Pick the cell count that puts this pack in a physically sane window.

    A 2S..8S pack at 3.2-4.25 V/cell is unambiguous for most voltages; where
    two counts both fit we take the higher (a nearly-flat 4S is a real state
    worth alarming about, a 6S at 4.4 V/cell is not a real state at all).
    """
    if BATT["cells"]:
        return BATT["cells"]
    if not v_in or v_in < 5.0:
        return None
    best = None
    for n in range(2, 9):
        per = v_in / n
        if 3.20 <= per <= V_CELL_FULL:
            best = n
    if best:
        BATT["cells"] = best
        BATT["inferred"] = True
        print(f"[batt] pack looks like {best}S "
              f"({v_in:.1f} V = {v_in / best:.2f} V/cell). "
              f"warn {best * V_CELL_WARN:.1f} V, alarm {best * V_CELL_ALARM:.1f} V. "
              f"Set BATTERY_CELLS in config.py to pin it.", flush=True)
    return best


def _session_info():
    """Which code is flying. A screenshot of a run is worth very little if you
    cannot tell afterwards which commit produced it, so the commit — and
    whether the tree was dirty — is on the screen the whole time."""
    sha, dirty = "unknown", False
    try:
        import subprocess
        sha = subprocess.check_output(
            ["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL, timeout=3).decode().strip()
        dirty = bool(subprocess.check_output(
            ["git", "-C", ROOT, "status", "--porcelain"],
            stderr=subprocess.DEVNULL, timeout=3).decode().strip())
    except Exception:                                        # noqa: BLE001
        pass
    return {"sha": sha, "dirty": dirty,
            "started": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "epoch_ms": int(time.time() * 1000)}


# Snapshot of the constants the cockpit draws with. Sent once per poll so the
# front end never carries its own copy of a calibration number; "est" names the
# ones that are still guesses, and the DIAGNOSE screen marks them as such.
CONFIG_SNAPSHOT = {
    "wheelbase": config.WHEELBASE_M,
    "maxSteer": config.MAX_STEER_ANGLE_RAD,
    "lookahead": LOOKAHEAD_M,
    "goalTol": GOAL_TOL_M,
    "steerGain": STEER_GAIN,
    "react": REACT_M,
    "forwardDeg": config.LIDAR_FORWARD_DEG,
    "maxDuty": config.MAX_DUTY,
    "mpt": config.METERS_PER_TACH,
    "est": ["WHEELBASE_M", "MAX_STEER_ANGLE_RAD"],
}


def _jpeg(img, quality=70):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def _placeholder(text):
    img = np.zeros((360, 480, 3), np.uint8)
    img[:] = (14, 18, 22)
    cv2.putText(img, text, (24, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (90, 120, 130), 1, cv2.LINE_AA)
    return _jpeg(img)


class Hub:
    def __init__(self):
        self.mode = "drive"
        self.env = "indoor"
        self.running = False

        # hardware (light core, always on)
        self.lidar = None
        self.vesc = None
        self.steer = None
        self.v_lock = threading.Lock()
        self.nav = LidarNavigator(react_m=REACT_M)

        # cameras
        self.front = None
        self.rear = None
        self.rear_on = False

        # perception (loaded on demand)
        self.detector = None
        self.front_dets = []
        self.rear_dets = []

        # SLAM / navigate
        self.slam = None
        self.nav_grid = None
        self.nav_trav = None
        self.nav_blocked = None
        self.nav_base = None       # pre-tinted map base (BGR)
        self.loc = None
        self.last_scan = None
        self.last_scan_t = 0.0          # when last_scan arrived (staleness guard)
        self.near_m = None               # nearest obstacle ahead, metres
        self.loop_ms = None              # measured control-loop period

        # GNSS. Outdoor-only by nature: indoors this reports fix=False, which
        # is the truth, rather than a degraded position the operator might act
        # on. Nothing in the control path depends on it.
        self.gps = None
        self.session = _session_info()
        self._trail = []                 # recent world-frame poses (breadcrumbs)

        # --- session recording (see recording/README.md) --------------------
        # The recorder owns no hardware: the loops below feed it, so recording
        # can never contend for a serial port. OFF by default — recording is a
        # deliberate act, not a background cost.
        self.rec = None
        self.rec_lock = threading.Lock()
        self._scan_arrival = 0.0     # arrival time of the newest scan we logged
        self.scan_hz = None          # MEASURED revolution rate (not 1/scan_age)
        self._rec_scan_seq = None    # links each telemetry row to a revolution

        # frame buffers (jpeg bytes)
        self.front_jpg = _placeholder("FRONT CAMERA")
        self.rear_jpg = _placeholder("REAR CAMERA — off")
        self.map_jpg = _placeholder("MAP")

    # ------------------------------------------------------------------ #
    #  start / stop
    # ------------------------------------------------------------------ #
    def start(self):
        print("bringing up LiDAR ...")
        try:
            self.lidar = ThreadedLidar().start()
        except Exception as e:  # noqa: BLE001
            print("  LiDAR n/a:", e)
        from drivers.vesc import VESC, resolve_port
        try:
            if os.path.exists(resolve_port()):
                self.vesc = VESC()
                print("  VESC up.")
            else:
                print("  VESC n/a (motor battery off?) — telemetry/drive disabled.")
        except Exception as e:  # noqa: BLE001
            print("  VESC n/a:", e)
        try:
            from drivers.steering import ServoController
            self.steer = ServoController()
            self.steer.center(read_reply=False)
            print("  steering centered.")
        except Exception as e:  # noqa: BLE001
            print("  steering n/a:", e)
        try:
            self.front = Camera("front").start()
            print("  front camera up.")
        except Exception as e:  # noqa: BLE001
            print("  front camera n/a:", e)

        if ThreadedGPS is not None:
            try:
                self.gps = ThreadedGPS().start()
            except Exception as e:  # noqa: BLE001
                print("  GNSS n/a:", e)

        self.running = True
        for fn in (self._lidar_loop, self._front_loop, self._rear_loop,
                   self._map_loop, self._actuator_loop, self._follow_loop,
                   self._telemetry_loop):
            threading.Thread(target=self._guard(fn), daemon=True).start()
        print(f"\nMISSION CONTROL up:  http://localhost:{HTTP_PORT}  "
              f"(or http://<jetson-ip>:{HTTP_PORT})\nCtrl-C to stop.")
        return self

    def stop(self):
        self.running = False
        FOLLOW["on"] = False
        time.sleep(0.2)
        if self.vesc:
            try:
                self.vesc.set_duty(0.0); self.vesc.stop(); self.vesc.close()
            except Exception:
                pass
        if self.steer:
            try:
                self.steer.center(); self.steer.close()
            except Exception:
                pass
        if self.front:
            try:
                self.front.release()
            except Exception:
                pass
        if self.rear:
            try:
                self.rear.release()
            except Exception:
                pass
        if self.lidar:
            try:
                self.lidar.stop()
            except Exception:
                pass
        if self.gps:
            try:
                self.gps.stop()
            except Exception:
                pass
        # Recorder LAST. Closing it can block for up to the writer-join timeout,
        # and nothing may sit between Ctrl-C and the motor being zeroed. The log
        # is flushed as it is written, so the only thing this adds is marking
        # meta.json complete — which is how you tell a full session from a
        # killed one.
        try:
            self.stop_recording()
        except Exception:                                    # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    #  mode switching (this is what keeps memory in budget)
    # ------------------------------------------------------------------ #
    def set_mode(self, mode):
        if mode not in ("drive", "map", "navigate", "perception") or mode == self.mode:
            return
        prev = self.mode
        # ---- tear down the mode we're leaving ----
        if prev == "perception":
            self._unload_detector()
        if prev == "map":
            pass    # keep the slam object so a partial map survives a peek elsewhere
        # ---- set up the mode we're entering ----
        if mode == "map":
            if self.slam is None:
                self.slam = LidarSLAM()
        if mode == "navigate":
            self._load_nav_map()
        if mode == "perception":
            self._load_detector()
        self.mode = mode
        FOLLOW["on"] = False
        print(f"[mode] {prev} -> {mode}")

    def _load_detector(self):
        if self.detector is not None:
            return
        try:
            print("[perception] loading YOLO detector ...")
            from perception.detect import Detector
            self.detector = Detector()
            print(f"[perception] detector ready ({self.detector.kind}).")
        except Exception as e:  # noqa: BLE001
            print("[perception] detector load FAILED:", e)
            self.detector = None

    def _unload_detector(self):
        if self.detector is None:
            return
        print("[perception] freeing detector ...")
        self.detector = None
        self.front_dets = []
        self.rear_dets = []
        gc.collect()
        try:
            import torch
            torch.cuda.empty_cache()
        except Exception:
            pass

    def _load_nav_map(self):
        if not os.path.exists(MAP_PATH):
            self.nav_grid = None
            return
        self.nav_grid = np.load(MAP_PATH)
        self.nav_trav = P.traversable_mask(self.nav_grid)
        self.nav_blocked = P.obstacle_mask(self.nav_grid)
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.nav_grid))
        base = ((1.0 - prob) * 255).astype(np.uint8)
        base = np.flipud(base)
        base = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
        base = cv2.resize(base, (OUT, OUT), interpolation=cv2.INTER_NEAREST)
        trav_img = cv2.resize((np.flipud(self.nav_trav).astype(np.uint8) * 255),
                              (OUT, OUT), interpolation=cv2.INTER_NEAREST)
        tint = trav_img > 0
        base[tint] = (0.55 * base[tint] + np.array([50, 90, 40])).clip(0, 255).astype(np.uint8)
        self.nav_base = base
        self.loc = P.Localizer(self.nav_grid)

    # ------------------------------------------------------------------ #
    #  loops
    # ------------------------------------------------------------------ #
    def _guard(self, fn):
        """Run a loop thread so that a crash is LOUD instead of silent.

        These threads are daemons: if one raised, it died quietly and the only
        symptom was behaviour going missing — auto-drive that stops steering, a map
        that stops growing. The deadman still cuts throttle (the actuator loop stops
        being fed), but you'd have no idea why. Now it says so, and forces a stop.
        """
        import traceback

        def runner():
            try:
                fn()
            except Exception:
                print(f"\n*** LOOP CRASHED: {fn.__name__} ***", flush=True)
                traceback.print_exc()
                FOLLOW["on"] = False
                FOLLOW["note"] = f"stopped: {fn.__name__} crashed"
                with C_LOCK:
                    CTRL["throttle"] = 0.0
                    CTRL["armed"] = False
                print("*** throttle disarmed. Fix the traceback above. ***\n",
                      flush=True)
        runner.__name__ = f"guard_{fn.__name__}"
        return runner

    def _lidar_loop(self):
        while self.running:
            if not self.lidar:
                time.sleep(0.2); continue
            scan, age = self.lidar.latest()
            if scan and age < 1.0:
                now = time.monotonic()
                # latest() re-serves the SAME revolution between sensor updates
                # (~10 Hz sensor, 20 Hz poll). Recording it twice would claim a
                # scan rate the C1 cannot deliver, so log only genuinely new
                # revolutions — identified by their arrival time, not poll time.
                arrival = now - age
                is_new = arrival > self._scan_arrival + 1e-6
                # Measure the ACTUAL revolution rate from the interval between
                # new scans. The cockpit used to derive Hz as 1/scan_age, which
                # is not the scan rate at all — it is how fresh the newest scan
                # happens to be at the moment you look, so a fast poll made a
                # 10 Hz scanner read 25 Hz.
                if is_new and self._scan_arrival:
                    dt = arrival - self._scan_arrival
                    if 0.005 < dt < 2.0:
                        self.scan_hz = (dt and 1.0 / dt) if self.scan_hz is None \
                            else 0.8 * self.scan_hz + 0.2 / dt
                self._scan_arrival = arrival
                self.last_scan = scan
                self.last_scan_t = now
                rec = self.rec
                if is_new and rec is not None:
                    try:
                        # Stamp with when the scan ARRIVED, not when we noticed it.
                        self._rec_scan_seq = rec.log_scan(scan, t=rec.now() - age)
                    except Exception:                        # noqa: BLE001
                        pass
                # Nearest obstacle ahead, published for the UI's clearance tile. Computed
                # here (once per scan) rather than in the follow loop, so it is available
                # in every mode and never derives from a scan the UI can't age-check.
                try:
                    self.near_m = self.nav.plan(scan)["nearest_ahead_m"]
                except Exception:
                    self.near_m = None
                if self.mode == "map" and self.slam is not None:
                    self.slam.add_scan(scan)
                if self.mode == "navigate" and self.loc is not None:
                    self.loc.update(scan)
                    pose = self.loc.pose
                    with S_LOCK:
                        STATE["pose"] = [round(v, 2) for v in pose]
                    # Breadcrumbs. Bounded on purpose: this is drawn every
                    # frame and serialised every poll, so it is a display
                    # buffer, not a log. The log is recording/, which keeps
                    # every sample and does not have to be cheap.
                    if (not self._trail or
                            math.hypot(pose[0] - self._trail[-1][0],
                                       pose[1] - self._trail[-1][1]) > 0.05):
                        self._trail.append((pose[0], pose[1]))
                        if len(self._trail) > 400:
                            del self._trail[0]
            time.sleep(0.05)

    def _front_loop(self):
        seq = 0
        while self.running:
            if not self.front:
                time.sleep(0.2); continue
            frame, seq = self.front.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            frame = frame.copy()
            if self.mode == "perception" and self.detector is not None:
                try:
                    dets = self.detector.detect(frame)
                    self.front_dets = dets
                    self.detector.draw(frame, dets)
                    h_, w_ = frame.shape[:2]
                    with S_LOCK:
                        # normalised boxes: the browser scales the video to fit
                        # its panel, so pixel coordinates from a 640x480 frame
                        # would land in the wrong place on every screen size
                        STATE["front_dets"] = [
                            [round(d["box"][0] / w_, 4), round(d["box"][1] / h_, 4),
                             round(d["box"][2] / w_, 4), round(d["box"][3] / h_, 4),
                             d["name"], round(d["conf"], 2)]
                            for d in dets]
                except Exception as e:  # noqa: BLE001
                    print("[perception] detect error:", e)
            self._hud(frame, "FRONT")
            jpg = _jpeg(frame)
            if jpg:
                self.front_jpg = jpg

    def _rear_loop(self):
        seq = 0
        while self.running:
            if not self.rear_on:
                time.sleep(0.15); continue
            if self.rear is None:
                try:
                    self.rear = Camera("rear").start()
                except Exception as e:  # noqa: BLE001
                    print("  rear camera n/a:", e)
                    self.rear_on = False
                    self.rear_jpg = _placeholder("REAR CAMERA — n/a")
                    continue
            frame, seq = self.rear.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            frame = frame.copy()
            if self.mode == "perception" and self.detector is not None:
                try:
                    dets = self.detector.detect(frame)
                    self.rear_dets = dets
                    self.detector.draw(frame, dets)
                except Exception:
                    pass
            self._hud(frame, "REAR")
            jpg = _jpeg(frame)
            if jpg:
                self.rear_jpg = jpg
        # rear turned off -> free the device
        if self.rear is not None:
            try:
                self.rear.release()
            except Exception:
                pass
            self.rear = None
            self.rear_jpg = _placeholder("REAR CAMERA — off")

    def _hud(self, frame, tag):
        h, w = frame.shape[:2]
        cv2.putText(frame, tag, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (95, 211, 188), 2, cv2.LINE_AA)
        cv2.rectangle(frame, (0, 0), (w - 1, h - 1), (40, 60, 68), 1)

    def _map_loop(self):
        while self.running:
            try:
                if self.mode == "map" and self.slam is not None:
                    png = self.slam.render_png(out_size=OUT)
                    if png:
                        self.map_jpg = png
                    with S_LOCK:
                        STATE["frames"] = self.slam.frames
                        STATE["pose"] = [round(v, 2) for v in self.slam.pose]
                elif self.mode == "navigate":
                    self.map_jpg = self._render_navigate()
                else:  # drive / perception -> live radar
                    self.map_jpg = self._render_radar()
            except Exception as e:  # noqa: BLE001
                print("map loop:", e)
            time.sleep(0.18)

    def _render_radar(self):
        img = np.zeros((OUT, OUT, 3), np.uint8)
        img[:] = (10, 14, 18)
        cx = cy = OUT // 2
        # range rings + crosshair
        for r_m in (1, 2, 3, 4, 5):
            cv2.circle(img, (cx, cy), int(r_m * RADAR_SCALE), (28, 42, 48), 1)
        cv2.line(img, (cx, 0), (cx, OUT), (24, 36, 42), 1)
        cv2.line(img, (0, cy), (OUT, cy), (24, 36, 42), 1)
        scan = self.last_scan
        fwd = math.radians(config.LIDAR_FORWARD_DEG)
        if scan:
            for _q, ang, dmm in scan:
                d = dmm / 1000.0
                if d < config.LIDAR_MIN_M or d > 6.0:
                    continue
                # rotate so the car's forward points UP on screen
                a = math.radians(ang) - fwd - math.pi / 2.0
                px = int(cx + d * RADAR_SCALE * math.cos(a))
                py = int(cy + d * RADAR_SCALE * math.sin(a))
                if 0 <= px < OUT and 0 <= py < OUT:
                    col = (90, 210, 190) if d > config.LIDAR_STOP_M else (70, 90, 255)
                    cv2.circle(img, (px, py), 2, col, -1)
        # car marker + forward arrow (up)
        cv2.circle(img, (cx, cy), 6, (95, 211, 188), -1)
        cv2.arrowedLine(img, (cx, cy), (cx, cy - 26), (95, 211, 188), 2, tipLength=0.4)
        cv2.putText(img, "LIDAR RADAR (top-down)", (10, OUT - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (70, 100, 110), 1, cv2.LINE_AA)
        return _jpeg(img)

    def _render_navigate(self):
        if self.nav_base is None:
            return _placeholder("NAVIGATE — no saved map (use MAP mode)")
        img = self.nav_base.copy()
        cells = PATH["cells"]
        if cells:
            poly = []
            for (r, c) in cells[::3]:
                x, y = P.cell_to_world(r, c)
                px, py = LidarSLAM.world_to_px(x, y, OUT)
                poly.append([int(px), int(py)])
            if len(poly) > 1:
                cv2.polylines(img, [np.array(poly, np.int32)], False, (80, 220, 80), 2)
        if GOAL["cell"]:
            gx, gy = P.cell_to_world(*GOAL["cell"])
            px, py = LidarSLAM.world_to_px(gx, gy, OUT)
            cv2.drawMarker(img, (int(px), int(py)), (60, 60, 255),
                           cv2.MARKER_TILTED_CROSS, 16, 2)
        with S_LOCK:
            pose = STATE.get("pose", [0, 0, 0])
        px, py = LidarSLAM.world_to_px(pose[0], pose[1], OUT)
        cv2.circle(img, (int(px), int(py)), 6, (230, 150, 60), -1)
        return _jpeg(img)

    # ------------------------------------------------------------------ #
    #  actuation + auto-drive
    # ------------------------------------------------------------------ #
    def _actuator_loop(self):
        duty = 0.0
        prev = None
        while self.running:
            now = time.monotonic()
            if prev is not None:
                # EMA of the real loop period, so the UI can show latency rather than
                # the operator having to assume it.
                dt = (now - prev) * 1000.0
                self.loop_ms = dt if self.loop_ms is None else 0.8 * self.loop_ms + 0.2 * dt
            prev = now
            with C_LOCK:
                estop = CTRL["estop"]; armed = CTRL["armed"]
                target = CTRL["throttle"]
                steer_cmd = CTRL["steer"]
                fresh = (now - CTRL["last_cmd"]) < DEADMAN_S
            if estop or not armed or not fresh:
                target = 0.0
            target = max(-MAXD, min(MAXD, target))
            duty = min(target, duty + RAMP) if duty < target else max(target, duty - RAMP)
            if self.vesc:
                with self.v_lock:
                    try:
                        self.vesc.set_duty(duty)
                    except Exception:
                        pass
            with S_LOCK:
                STATE["drive"] = {"armed": armed, "estop": estop,
                                  "duty": round(duty * 100, 1)}
                tele = STATE.get("tele") or {}
                pose = STATE.get("pose") or (None, None, None)

            # --- session recording -------------------------------------------
            # This loop is the right place: it is the only one that sees the
            # ACTUAL commanded duty (post deadman, post cap, post ramp) at the
            # 20 Hz control rate. VESC telemetry refreshes at ~3 Hz, so those
            # columns are carried forward — documented in recording/README.md.
            rec = self.rec
            if rec is not None:
              try:
                rec.log({
                    "cmd_duty": duty, "cmd_steer": steer_cmd,
                    "mode": self.mode, "armed": armed, "estop": estop,
                    "follow_on": FOLLOW["on"],
                    "erpm": tele.get("erpm"), "tach": tele.get("tach"),
                    "v_in": tele.get("v_in"),
                    "motor_current": tele.get("motor_current"),
                    "temp_mos": tele.get("temp_mos"),
                    "temp_motor": tele.get("temp_motor"),
                    "fault": tele.get("fault"), "speed_mps": tele.get("speed"),
                    "pose_x": pose[0], "pose_y": pose[1], "pose_yaw": pose[2],
                    "scan_seq": self._rec_scan_seq,
                    "scan_age": (now - self.last_scan_t) if self.last_scan_t else None,
                    "near_m": self.near_m,
                })
              except Exception:                              # noqa: BLE001
                pass
            time.sleep(0.05)

    # ------------------------------------------------------------------ #
    #  session recording
    # ------------------------------------------------------------------ #
    def start_recording(self, note=""):
        """Begin a session. Idempotent; never raises into the caller."""
        if Recorder is None:
            return False
        with self.rec_lock:
            if self.rec is not None:
                return True
            try:
                self.rec = Recorder(note=note, source="cockpit").start()
                self._rec_scan_seq = None
                return True
            except Exception as e:                          # noqa: BLE001
                print("  [rec] start failed:", e)
                self.rec = None
                return False

    def stop_recording(self):
        with self.rec_lock:
            rec, self.rec = self.rec, None
        if rec is None:
            return None
        try:
            return rec.stop()
        except Exception as e:                              # noqa: BLE001
            print("  [rec] stop failed:", e)
            return None

    def rec_status(self):
        rec = self.rec
        if rec is None:
            return {"on": False}
        try:
            st = rec.stats()
            return {"on": True, "rows": st["telemetry"], "scans": st["scans"],
                    "secs": st["duration_s"], "dropped": st["dropped"],
                    "name": os.path.basename(st["dir"])}
        except Exception:                                    # noqa: BLE001
            return {"on": True}

    def set_steer(self, v):
        v = max(-1.0, min(1.0, v))
        with C_LOCK:
            CTRL["steer"] = v
        if self.steer:
            try:
                self.steer.steer(v, read_reply=False)
            except Exception:
                pass

    def _drive(self, throttle, steer):
        with C_LOCK:
            if throttle != 0.0:
                CTRL["armed"] = True
            CTRL["throttle"] = throttle
            CTRL["last_cmd"] = time.monotonic()
        self.set_steer(steer)

    def set_goal(self, col_px, row_px):
        if self.nav_trav is None:
            return
        wx, wy = LidarSLAM.px_to_world(col_px, row_px, OUT)
        goal = P.snap_to_traversable(self.nav_trav, P.world_to_cell(wx, wy))
        with S_LOCK:
            pose = STATE.get("pose", [0.0, 0.0, 0.0])
        start = P.snap_to_traversable(~self.nav_blocked, P.world_to_cell(pose[0], pose[1]))
        path = P.astar(self.nav_blocked, start, goal) if (start and goal) else None
        print(f"[goal] px=({col_px:.0f},{row_px:.0f}) world=({wx:.2f},{wy:.2f}) "
              f"goal={goal} start={start} "
              f"path={'None' if path is None else str(len(path)) + ' cells'}", flush=True)
        GOAL["cell"] = goal
        PATH["cells"] = path
        PATH["world"] = [P.cell_to_world(r, c) for (r, c) in path] if path else None
        FOLLOW["on"] = False; FOLLOW["arrived"] = False; FOLLOW["note"] = ""
        with S_LOCK:
            STATE["goal"] = [round(wx, 2), round(wy, 2)]
            STATE["path_len"] = len(path) if path else 0
            STATE["reachable"] = path is not None

    def _replan(self):
        if not GOAL["cell"]:
            return
        with S_LOCK:
            pose = STATE.get("pose", [0.0, 0.0, 0.0])
        start = P.snap_to_traversable(~self.nav_blocked, P.world_to_cell(pose[0], pose[1]))
        path = P.astar(self.nav_blocked, start, GOAL["cell"]) if start else None
        if path:
            PATH["cells"] = path
            PATH["world"] = [P.cell_to_world(r, c) for (r, c) in path]

    def _pursuit_target(self, x, y):
        wp = PATH["world"]
        if not wp:
            return None
        di = [(px - x) ** 2 + (py - y) ** 2 for (px, py) in wp]
        i = int(min(range(len(wp)), key=lambda k: di[k]))
        tx, ty = wp[-1]; acc = 0.0
        for j in range(i, len(wp) - 1):
            ax, ay = wp[j]; bx, by = wp[j + 1]
            acc += math.hypot(bx - ax, by - ay)
            if acc >= LOOKAHEAD_M:
                tx, ty = bx, by; break
        gx, gy = wp[-1]
        return tx, ty, math.hypot(gx - x, gy - y)

    def _follow_loop(self):
        last_plan = 0.0
        blocked_since = None
        while self.running:
            time.sleep(0.08)
            if not FOLLOW["on"] or self.mode != "navigate":
                blocked_since = None
                continue
            with C_LOCK:
                estop = CTRL["estop"]
            if estop:
                FOLLOW["on"] = False; FOLLOW["note"] = "E-STOP"; continue
            now = time.monotonic()
            if now - last_plan >= REPLAN_EVERY_S:
                last_plan = now; self._replan()
            # --- SAFETY GATES (both fail closed) -------------------------- #
            # A scan that stopped arriving used to sit in last_scan forever, still
            # reading "clear", so a dead LiDAR meant driving on blind. Age it.
            scan = self.last_scan
            scan_age = time.monotonic() - self.last_scan_t
            if scan is None or scan_age > SCAN_MAX_AGE_S:
                self._drive(0.0, 0.0)
                FOLLOW["note"] = f"stopped: no LiDAR ({scan_age:.1f}s stale)"
                continue
            # A diverged ICP pose is worse than none — don't steer on one.
            if self.loc is None or not self.loc.healthy():
                self._drive(0.0, 0.0)
                FOLLOW["note"] = "stopped: lost localization"
                continue

            with S_LOCK:
                pose = STATE.get("pose", [0.0, 0.0, 0.0])
            x, y, th = pose[0], pose[1], pose[2]
            tgt = self._pursuit_target(x, y)
            if tgt is None:
                self._drive(0.0, 0.0); FOLLOW["note"] = "no route"; continue
            tx, ty, dgoal = tgt
            if dgoal < GOAL_TOL_M:
                self._drive(0.0, 0.0); FOLLOW["on"] = False
                FOLLOW["arrived"] = True; FOLLOW["note"] = "arrived"
                print("[follow] ARRIVED.", flush=True); continue
            desired = math.atan2(ty - y, tx - x)
            car_heading = th + math.radians(FOLLOW_FORWARD_DEG)
            err = (desired - car_heading + math.pi) % (2 * math.pi) - math.pi
            steer = max(-1.0, min(1.0, FOLLOW_STEER_SIGN * STEER_GAIN * err))
            pursuit_term = steer
            veto_term = None
            # scan is guaranteed fresh by the gate above. Both steer terms now share
            # config.STEER_SIGN, so blending them can no longer cancel out.
            nav = self.nav.plan(scan)
            blocked = nav["blocked"]; near = nav["nearest_ahead_m"]
            if not blocked and near < REACT_M:
                veto_term = nav["steer"]
                steer = max(-1.0, min(1.0, 0.5 * steer + 0.5 * nav["steer"]))
            duty = DUTY_OUTDOOR if self.env == "outdoor" else DUTY_INDOOR
            if blocked:
                self._drive(0.0, steer)
                blocked_since = blocked_since or now
                FOLLOW["note"] = f"blocked {near:.2f}m — waiting"
                if now - blocked_since > BLOCK_GIVEUP_S:
                    self._drive(0.0, 0.0); FOLLOW["on"] = False
                    FOLLOW["note"] = "stopped: blocked too long"
            else:
                blocked_since = None
                self._drive(duty, steer)
                FOLLOW["note"] = f"driving {dgoal:.2f}m to goal"
            # perpendicular distance to the route: the number that says whether
            # the car is ON the plan, as opposed to merely heading toward it
            xtrack = None
            wp = PATH.get("world")
            if wp:
                xtrack = min(math.hypot(px - x, py - y) for (px, py) in wp)
            with S_LOCK:
                STATE["follow"] = {"on": FOLLOW["on"], "arrived": FOLLOW["arrived"],
                                   "note": FOLLOW["note"],
                                   "heading_err": round(math.degrees(err), 2),
                                   "cross_track": (round(xtrack, 3)
                                                   if xtrack is not None else None),
                                   "steer_pursuit": round(pursuit_term, 3),
                                   "steer_veto": (round(veto_term, 3)
                                                  if veto_term is not None else 0.0),
                                   "icp_fail": getattr(self.loc, "fails", None)}

    # ------------------------------------------------------------------ #
    #  telemetry
    # ------------------------------------------------------------------ #
    def _telemetry_loop(self):
        last_tach = None; last_t = None
        while self.running:
            tele = {}
            if self.vesc:
                try:
                    with self.v_lock:
                        v = self.vesc.get_values()
                except Exception:
                    v = None
                if v:
                    now = time.monotonic()
                    speed = 0.0
                    if last_tach is not None and last_t is not None and now > last_t:
                        speed = (v["tach"] - last_tach) * config.METERS_PER_TACH / (now - last_t)
                    last_tach, last_t = v["tach"], now
                    tele = {"v_in": round(v["v_in"], 1),
                            "temp_mos": round(v["temp_mos"], 1),
                            "temp_motor": round(v["temp_motor"], 1),
                            "motor_current": round(v["motor_current"], 1),
                            "erpm": v["erpm"], "tach": v["tach"],
                            "fault": v["fault_name"],
                            "speed": round(abs(speed), 2)}
            heading = None
            if self.mode in ("navigate",):
                with S_LOCK:
                    pose = STATE.get("pose")
                if pose:
                    heading = round((math.degrees(pose[2]) + 360) % 360, 0)
            scan_age = None
            if self.last_scan_t:
                scan_age = round(time.monotonic() - self.last_scan_t, 2)
            with S_LOCK:
                STATE["scan_age"] = scan_age
                STATE["near"] = (round(self.near_m, 2)
                                 if isinstance(self.near_m, float) and self.near_m != float("inf")
                                 else None)
                STATE["loop_ms"] = round(self.loop_ms, 1) if self.loop_ms else None
                STATE["mode"] = self.mode
                STATE["env"] = self.env
                STATE["rear_on"] = self.rear_on
                STATE["tele"] = tele
                STATE["rec"] = self.rec_status()
                if heading is not None:
                    STATE["heading"] = heading
                lidar_ok = False
                if self.lidar:
                    _s, age = self.lidar.latest()
                    lidar_ok = _s is not None and age < 1.5
                STATE["health"] = {
                    "loc": (self.loc.healthy() if self.loc is not None else False),
                    "lidar": lidar_ok, "vesc": self.vesc is not None,
                    "steer": self.steer is not None, "cam": self.front is not None,
                    "detector": self.detector is not None,
                }
                # The commanded side of every pair the cockpit draws. Without
                # this the front end can only show what the car DID, never what
                # it was ASKED to do — and the gap between those two is the
                # single most useful thing on the screen.
                with C_LOCK:
                    STATE["ctrl"] = {"armed": CTRL["armed"], "estop": CTRL["estop"],
                                     "throttle": round(CTRL["throttle"], 4),
                                     "steer": round(CTRL["steer"], 3)}
                STATE["gps"] = self.gps.snapshot() if self.gps else {
                    "fix": False, "present": False, "satlist": []}
                cells = _infer_cells(tele.get("v_in")) if tele else BATT["cells"]
                STATE["batt"] = ({"cells": cells, "inferred": BATT["inferred"],
                                  "warn": round(cells * V_CELL_WARN, 2),
                                  "alarm": round(cells * V_CELL_ALARM, 2),
                                  "full": round(cells * V_CELL_FULL, 2),
                                  "v_cell": (round(tele["v_in"] / cells, 2)
                                             if tele.get("v_in") else None)}
                                 if cells else {"cells": None})
                STATE["scan_hz"] = (round(self.scan_hz, 1)
                                    if self.scan_hz else None)
                STATE["session"] = self.session
                STATE["config"] = CONFIG_SNAPSHOT
                STATE.update(self._local_frame(STATE.get("pose")))
            time.sleep(0.3)

    def _local_frame(self, pose):
        """World-frame nav geometry expressed in the CAR frame, in metres.

        The scenes are all track-up: forward is +x, and lateral is +y to the
        side the LiDAR calls positive. Doing this rotation on the server keeps
        one definition of "where the nose points" in the codebase instead of
        two that can drift apart.

        The pose is passed IN rather than read from STATE, because the only
        caller already holds S_LOCK and threading.Lock is not reentrant —
        re-acquiring it here would deadlock the telemetry thread on the first
        pass and freeze every number on the screen. Nothing in this method may
        take that lock.
        """
        out = {"path_local": None, "goal_local": None, "lookahead_local": None,
               "trail_local": None, "goal_dist": None}
        if not pose:
            return out
        x, y, th = pose[0], pose[1], pose[2]
        c, s = math.cos(-th), math.sin(-th)

        def to_car(px, py):
            dx, dy = px - x, py - y
            return [round(dx * c - dy * s, 3), round(dx * s + dy * c, 3)]

        wp = PATH.get("world")
        if wp:
            # thin it: 60 points is more than any 700-px scene can resolve, and
            # this payload is fetched several times a second
            step = max(1, len(wp) // 60)
            out["path_local"] = [to_car(px, py) for (px, py) in wp[::step]]
            gx, gy = wp[-1]
            out["goal_local"] = to_car(gx, gy)
            out["goal_dist"] = round(math.hypot(gx - x, gy - y), 2)
            tgt = self._pursuit_target(x, y)
            if tgt:
                out["lookahead_local"] = to_car(tgt[0], tgt[1])
        if self._trail:
            out["trail_local"] = [to_car(px, py) for (px, py) in self._trail]
        return out

    def save_map(self):
        if self.slam is None:
            return None
        path = os.path.join(MAP_DIR, "room")
        self.slam.save(path)
        print("[map] saved ->", path)
        return path

    def scan_points(self):
        """Forward-referenced (x_up, y_right) points in metres for the 3D view."""
        scan = self.last_scan
        pts = []
        if scan:
            fwd = math.radians(config.LIDAR_FORWARD_DEG)
            for _q, ang, dmm in scan:
                d = dmm / 1000.0
                if d < config.LIDAR_MIN_M or d > 6.0:
                    continue
                a = math.radians(ang) - fwd          # 0 = straight ahead
                fx = d * math.cos(a)                  # forward
                fy = d * math.sin(a)                  # left/right
                pts.append([round(fx, 2), round(fy, 2)])
        return pts


HUB = None


PAGE = r"""<!doctype html><html lang=en><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>RoboCar Cockpit</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@600;700&family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
/* ═══ OPERATING PALETTE — per RoboCar HMI Style Guide §2 ══════════════════
   Colour appears only for abnormal conditions. Everything nominal is neutral. */
:root{
  --g:#e4e6e4; --s:#eff1ef; --s2:#e9ebe8;
  --struct:#a8ada8; --hair:#c6cbc5;
  --t1:#1a1f1d; --t2:#5a625e; --t3:#7d857f;
  --val:#2e3a44;
  --caution:#a8620a; --alarm:#b0231b; --override:#7a3f8a;
  --caution-f:#f4e3cb; --alarm-f:#f6dcd9;
  --estop-face:#b0231b; --estop-ring:#d8b400;
  --sel-bg:#1a1f1d; --sel-fg:#e4e6e4;
  --scene:#dcdfdb;
}
:root[data-hmi="night"]{
  --g:#14181a; --s:#1c2226; --s2:#191f22;
  --struct:#4a5559; --hair:#2b3438;
  --t1:#dee4e2; --t2:#94a3a3; --t3:#778586;
  --val:#c3d2da;
  --caution:#e0a53c; --alarm:#ff6b6b; --override:#c08bd4;
  --caution-f:#2e2410; --alarm-f:#33191a;
  --estop-face:#c22b21; --estop-ring:#d8b400;
  --sel-bg:#dee4e2; --sel-fg:#14181a;
  --scene:#101517;
}
*{box-sizing:border-box}
html,body{height:100%}
body{margin:0;background:var(--g);color:var(--t1);
  font-family:"IBM Plex Sans",system-ui,sans-serif;font-size:14px;line-height:1.45;
  -webkit-font-smoothing:antialiased;overflow:hidden}
.num,.mono{font-family:"IBM Plex Mono",ui-monospace,Menlo,monospace;
  font-variant-numeric:tabular-nums lining-nums slashed-zero;
  font-variant-ligatures:none}
button{font:inherit;color:inherit;border:0;background:none;cursor:pointer}
button:focus-visible,[tabindex]:focus-visible{outline:2px solid var(--t1);outline-offset:2px}
:root[data-hmi="night"] button:focus-visible{outline-color:var(--t1)}
.lab{font-family:"IBM Plex Mono",monospace;font-size:9px;letter-spacing:.15em;
  text-transform:uppercase;color:var(--t2)}

/* ═══ SHELL ═══ */
.shell{height:100vh;display:grid;grid-template-rows:auto auto 1fr auto;
  background:var(--hair);gap:1px}

/* ═══ STATUS BAR — persistent, every mode ═══ */
.bar{background:var(--s);display:flex;align-items:stretch;gap:1px;min-height:56px}
.bar>*{display:flex;align-items:center}
.brand{padding:0 16px;font-family:Archivo,sans-serif;font-weight:700;font-size:15px;
  letter-spacing:.02em;gap:9px;border-right:1px solid var(--hair)}
.brand .dot{width:7px;height:7px;background:var(--t1);border-radius:50%}

/* autonomy enum — named states, never a boolean */
.auto{padding:0 14px;gap:2px;border-right:1px solid var(--hair)}
.auto .seg{display:flex;border:1px solid var(--struct)}
.auto .seg button{padding:6px 11px;font-family:"IBM Plex Mono",monospace;font-size:10px;
  letter-spacing:.1em;color:var(--t2);min-height:30px}
.auto .seg button[aria-pressed="true"]{background:var(--sel-bg);color:var(--sel-fg);font-weight:600}
.auto .seg button.ov[aria-pressed="true"]{background:var(--override);color:#fff}

/* the one top-level state word */
.state{padding:0 18px;gap:11px;border-right:1px solid var(--hair);min-width:230px}
.state .gl{font-size:15px;line-height:1}
.state .w{font-family:Archivo,sans-serif;font-weight:700;font-size:17px;letter-spacing:.03em}
.state .sub{font-size:10.5px;color:var(--t2);line-height:1.25}
.state.caution{color:var(--caution)} .state.alarm{color:var(--alarm)}
.state.caution .sub,.state.alarm .sub{color:inherit;opacity:.85}

/* subsystem chips — redundant coding: glyph + text + border */
.chips{flex:1;padding:0 12px;gap:6px;flex-wrap:wrap;min-width:0}
.chip{display:inline-flex;align-items:center;gap:5px;padding:4px 8px;min-height:26px;
  border:1px solid var(--struct);font-family:"IBM Plex Mono",monospace;font-size:9.5px;
  letter-spacing:.07em;color:var(--t2);white-space:nowrap}
.chip .gl{font-size:10px;line-height:1}
.chip.caution{border-color:var(--caution);color:var(--caution);background:var(--caution-f)}
.chip.alarm{border-color:var(--alarm);color:var(--alarm);background:var(--alarm-f);border-width:2px}
.chip.stale{border-style:dashed;color:var(--t3)}

.barR{gap:1px;border-left:1px solid var(--hair)}
.themebtn{padding:0 13px;font-family:"IBM Plex Mono",monospace;font-size:9.5px;
  letter-spacing:.11em;color:var(--t2);align-self:stretch}
.themebtn:hover{color:var(--t1)}
.themebtn.reccing{color:#ff5b5b;border-color:#ff5b5b}
.themebtn.reccing::before{content:"";display:inline-block;width:6px;height:6px;
  border-radius:50%;background:#ff5b5b;margin-right:6px;animation:recblink 1.4s infinite}
@keyframes recblink{0%,49%{opacity:1}50%,100%{opacity:.15}}

/* E-STOP — ISO 13850. Red on yellow, reserved. Never shrouded. */
.estopwrap{align-self:stretch;display:flex;flex-direction:column;justify-content:center;
  background:var(--estop-ring);padding:5px 6px;gap:3px}
.estop{background:var(--estop-face);color:#fff;font-family:Archivo,sans-serif;font-weight:700;
  font-size:13px;letter-spacing:.11em;padding:9px 22px;min-height:40px;min-width:150px;
  border:2px solid #7d1712}
.estop:hover{background:#8f1c15}
.estop.latched{animation:estopblink 1s steps(2) infinite}
@keyframes estopblink{50%{background:#7d1712}}
.span{font-family:"IBM Plex Mono",monospace;font-size:7.5px;letter-spacing:.08em;
  color:#4a3d00;text-align:center;line-height:1.2}
.clearbtn{background:var(--s);border:2px solid var(--caution);color:var(--caution);
  font-family:Archivo,sans-serif;font-weight:700;font-size:11px;letter-spacing:.09em;
  padding:8px 14px;min-height:38px;align-self:stretch;margin:5px 0}

/* ═══ PRE-ARM STRIP ═══ */
.prearm{background:var(--caution-f);color:var(--caution);border-top:1px solid var(--caution);
  padding:8px 16px;display:flex;align-items:center;gap:12px;font-family:"IBM Plex Mono",monospace;
  font-size:12px}
.prearm.clear{background:var(--s);color:var(--t2);border-top-color:var(--hair)}
.prearm .tag{font-size:9px;letter-spacing:.14em;border:1px solid currentColor;padding:2px 6px}
.prearm .more{margin-left:auto;font-size:10px;opacity:.8}

/* ═══ BODY ═══ */
.body{display:grid;grid-template-columns:52px 1fr 306px;gap:1px;min-height:0}
@media(max-width:900px){
  .body{grid-template-columns:52px 1fr}
  .side{display:none}
}

/* mode rail — vertical, task-named */
.rail{background:var(--s);display:flex;flex-direction:column;gap:1px}
.rail button{writing-mode:vertical-rl;transform:rotate(180deg);padding:16px 0;flex:1;
  font-family:"IBM Plex Mono",monospace;font-size:10px;letter-spacing:.18em;color:var(--t2);
  border-right:2px solid transparent}
.rail button[aria-pressed="true"]{background:var(--sel-bg);color:var(--sel-fg);font-weight:600}
.rail button:hover:not([aria-pressed="true"]){background:var(--s2);color:var(--t1)}

/* scene */
.stage{background:var(--scene);position:relative;min-height:0;display:flex;flex-direction:column}
.stage canvas,.stage img{flex:1;width:100%;min-height:0;object-fit:contain;display:block}
.stagehead{position:absolute;top:0;left:0;right:0;display:flex;align-items:center;gap:10px;
  padding:9px 13px;pointer-events:none}
.stagehead .t{font-family:Archivo,sans-serif;font-weight:600;font-size:11px;letter-spacing:.13em;
  text-transform:uppercase;color:var(--t2)}
.stagefoot{display:flex;align-items:center;gap:14px;padding:7px 13px;background:var(--s);
  border-top:1px solid var(--hair);flex-wrap:wrap}
.stagefoot .m{font-family:"IBM Plex Mono",monospace;font-size:10px;color:var(--t2);
  font-variant-numeric:tabular-nums;display:flex;gap:5px;align-items:baseline}
.stagefoot .m b{color:var(--t1);font-weight:500}
.stagefoot .m.stale b{color:var(--caution)}
.hint{margin-left:auto;font-size:10.5px;color:var(--t3)}
/* camera PiP — one big spatial view, one demotable inset (QGC convention) */
.pip{position:absolute;right:11px;bottom:52px;width:212px;border:1px solid var(--struct);
  background:var(--scene);display:flex;flex-direction:column}
.pip.hidden{display:none}
.pip img{width:100%;display:block;aspect-ratio:4/3;object-fit:cover;background:var(--s2)}
.pip .ph{display:flex;align-items:center;gap:6px;padding:4px 7px;background:var(--s);
  border-bottom:1px solid var(--hair)}
.pip .ph span{font-family:"IBM Plex Mono",monospace;font-size:8.5px;letter-spacing:.13em;
  color:var(--t2)}
.pip .ph button{margin-left:auto;font-family:"IBM Plex Mono",monospace;font-size:8.5px;
  letter-spacing:.1em;color:var(--t2);padding:2px 5px;border:1px solid var(--struct);min-height:20px}
.pip .ph button:hover{color:var(--t1);border-color:var(--t1)}
.camtoggle{position:absolute;right:11px;bottom:52px;font-family:"IBM Plex Mono",monospace;
  font-size:9.5px;letter-spacing:.11em;color:var(--t2);border:1px solid var(--struct);
  background:var(--s);padding:6px 10px;min-height:30px}
.camtoggle.hidden{display:none}

/* contextual action row — changes with the task */
.act{display:flex;gap:6px;margin-bottom:11px;flex-wrap:wrap}
.act button{flex:1;border:1px solid var(--struct);min-height:40px;padding:0 12px;
  font-family:"IBM Plex Mono",monospace;font-size:11px;letter-spacing:.1em;color:var(--t1)}
.act button:hover{border-color:var(--t1)}
.act .note{width:100%;font-size:10px;color:var(--t2);font-family:"IBM Plex Mono",monospace}
.act .note.ok{color:var(--t1)}
.seg2{display:flex;border:1px solid var(--struct);width:100%}
.seg2 button{flex:1;border:0;padding:8px 0;font-family:"IBM Plex Mono",monospace;font-size:10px;
  letter-spacing:.1em;color:var(--t2);min-height:34px}
.seg2 button[aria-pressed="true"]{background:var(--sel-bg);color:var(--sel-fg);font-weight:600}

/* detections — VRU classes get caution, nothing else is coloured */
.dets{border:1px solid var(--hair);max-height:168px;overflow-y:auto}
.dets .d{display:flex;justify-content:space-between;gap:8px;padding:5px 9px;
  border-bottom:1px solid var(--hair);font-family:"IBM Plex Mono",monospace;font-size:10.5px;
  color:var(--t2)}
.dets .d:last-child{border-bottom:0}
.dets .d b{color:var(--t1);font-weight:500}
.dets .d.vru{color:var(--caution);background:var(--caution-f)}
.dets .d.vru b{color:var(--caution)}
.dets .empty{padding:9px;font-size:10.5px;color:var(--t3)}

/* ═══ SIDE COLUMN ═══ */
.side{background:var(--s);display:flex;flex-direction:column;gap:1px;overflow-y:auto;
  background:var(--hair)}
.blk{background:var(--s);padding:12px 13px}
.blkhead{display:flex;align-items:baseline;gap:8px;margin-bottom:9px}
.blkhead h2{margin:0;font-family:Archivo,sans-serif;font-size:10.5px;font-weight:600;
  letter-spacing:.15em;text-transform:uppercase;color:var(--t2)}
.blkhead .n{margin-left:auto;font-family:"IBM Plex Mono",monospace;font-size:9.5px;color:var(--t3)}

/* telemetry tiles — value + unit + MEANING */
.tiles{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:var(--hair);
  border:1px solid var(--hair)}
.tile{background:var(--s);padding:8px 9px;border-left:3px solid transparent}
.tile .row{display:flex;align-items:baseline;gap:4px;margin-top:1px}
.tile .v{font-family:"IBM Plex Mono",monospace;font-size:21px;font-weight:500;color:var(--val);
  line-height:1.12;font-variant-numeric:tabular-nums lining-nums slashed-zero;
  min-width:4ch;text-align:right}
.tile .u{font-family:"IBM Plex Mono",monospace;font-size:9.5px;color:var(--t2)}
.tile .mean{font-size:10px;color:var(--t2);margin-top:3px;line-height:1.3;min-height:26px}
.tile.caution{border-left-color:var(--caution)} .tile.caution .v{color:var(--caution)}
.tile.alarm{border-left-color:var(--alarm)} .tile.alarm .v{color:var(--alarm)}
.tile.stale{border-style:dashed;border-color:var(--struct);border-left-color:var(--struct)}
.tile.stale .v{color:var(--t3)}

/* control */
.gov{display:flex;align-items:center;gap:8px;margin-bottom:11px}
.gov .cap{flex:1;font-family:"IBM Plex Mono",monospace;font-size:11px;color:var(--t1)}
.gov .cap b{font-size:16px;font-weight:600}
.gov button{border:1px solid var(--struct);width:34px;height:34px;font-size:16px;color:var(--t2)}
.gov button:hover{border-color:var(--t1);color:var(--t1)}

.pad{display:grid;grid-template-columns:1fr 1.15fr 1fr;grid-template-rows:auto auto auto;
  grid-template-areas:". u ." "l c r" ". d .";gap:5px}
.pad button{border:1px solid var(--struct);min-height:52px;font-size:19px;color:var(--t2);
  touch-action:none;-webkit-user-select:none;user-select:none}
.pad button:hover{border-color:var(--t1);color:var(--t1)}
.pad button.on{background:var(--sel-bg);color:var(--sel-fg);border-color:var(--sel-bg)}
.pad .c{grid-area:c;border:1px solid var(--hair);background:var(--s2);display:flex;
  flex-direction:column;align-items:center;justify-content:center;gap:1px;padding:4px}
.pad .c span{font-family:"IBM Plex Mono",monospace;font-size:10px;color:var(--t2);
  font-variant-numeric:tabular-nums}
.pad .c span b{color:var(--val);font-weight:500}

.trim{margin-top:11px}
.trim input{width:100%;accent-color:var(--t1);height:26px}

/* slide-to-confirm — one gesture for anything that moves the car */
.slider{margin-top:11px;position:relative;height:46px;border:1px solid var(--struct);
  background:var(--s2);overflow:hidden;touch-action:none;-webkit-user-select:none;user-select:none}
.slider .fill{position:absolute;inset:0;width:0;background:var(--sel-bg);opacity:.13}
.slider .txt{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;
  gap:7px;font-family:"IBM Plex Mono",monospace;font-size:11px;letter-spacing:.12em;
  color:var(--t2);pointer-events:none}
.slider .knob{position:absolute;top:3px;left:3px;bottom:3px;width:56px;background:var(--sel-bg);
  color:var(--sel-fg);display:flex;align-items:center;justify-content:center;font-size:15px;
  cursor:grab}
.slider[data-armed="1"] .knob{cursor:grabbing}
.slider.done .knob{background:var(--t2)}
.slider.disabled{opacity:.45;pointer-events:none}
.stopbtn{margin-top:6px;border:1px solid var(--struct);width:100%;min-height:38px;
  font-family:"IBM Plex Mono",monospace;font-size:11px;letter-spacing:.12em;color:var(--t2)}
.stopbtn:hover{border-color:var(--alarm);color:var(--alarm)}

.keys{margin-top:11px;padding-top:9px;border-top:1px solid var(--hair);font-size:10.5px;
  color:var(--t3);line-height:1.6}
.keys kbd{font-family:"IBM Plex Mono",monospace;font-size:10px;border:1px solid var(--struct);
  padding:1px 4px;color:var(--t2)}
.keys.warn{color:var(--caution)}

/* ═══ EVENT LOG — every transition, with its cause ═══ */
.log{background:var(--s);display:flex;align-items:center;gap:11px;padding:0 13px;
  min-height:34px;overflow:hidden}
.log .lab{flex-shrink:0}
.log ul{display:flex;gap:16px;margin:0;padding:0;list-style:none;overflow-x:auto;flex:1}
.log li{font-family:"IBM Plex Mono",monospace;font-size:10px;color:var(--t2);white-space:nowrap;
  display:flex;gap:6px;align-items:baseline}
.log li time{color:var(--t3)}
.log li.caution{color:var(--caution)} .log li.alarm{color:var(--alarm)}
.simtag{flex-shrink:0;font-family:"IBM Plex Mono",monospace;font-size:9px;letter-spacing:.13em;
  border:1px dashed var(--override);color:var(--override);padding:2px 7px}

@media (prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important}}
</style>

<div class="shell">

  <!-- ══ STATUS ══ -->
  <header class="bar">
    <div class="brand"><span class="dot"></span>ROBOCAR</div>

    <div class="auto">
      <div style="display:flex;flex-direction:column;gap:3px">
        <span class="lab">Autonomy</span>
        <div class="seg" role="group" aria-label="Autonomy level">
          <button id="m-manual" class="ov" aria-pressed="true">MANUAL</button>
          <button id="m-assist" aria-pressed="false">ASSISTED</button>
          <button id="m-auto" aria-pressed="false">AUTO</button>
        </div>
      </div>
    </div>

    <div class="state" id="state">
      <span class="gl" id="stateGl">●</span>
      <div><div class="w" id="stateW">READY</div><div class="sub" id="stateSub">disarmed · stand test</div></div>
    </div>

    <div class="chips" id="chips"></div>

    <div class="barR">
      <button class="themebtn" id="recbtn" title="Record this session to logs/">● REC</button>
      <button class="themebtn" id="themebtn">☾ NIGHT</button>
      <div class="estopwrap">
        <button class="estop" id="estop">■ E-STOP</button>
        <div class="span">STOPS: DRIVE MOTOR + STEERING</div>
      </div>
      <button class="clearbtn" id="clearstop" hidden>↺ CLEAR<br>E-STOP</button>
    </div>
  </header>

  <!-- ══ PRE-ARM ══ -->
  <div class="prearm" id="prearm">
    <span class="tag">PRE-ARM</span>
    <span id="prearmMsg" class="mono">checking…</span>
    <span class="more" id="prearmMore"></span>
  </div>

  <!-- ══ BODY ══ -->
  <div class="body">
    <nav class="rail" role="group" aria-label="Task">
      <button id="t-drive" aria-pressed="true">DRIVE</button>
      <button id="t-map" aria-pressed="false">MAP</button>
      <button id="t-navigate" aria-pressed="false">NAVIGATE</button>
      <button id="t-perception" aria-pressed="false">PERCEPTION</button>
    </nav>

    <main class="stage">
      <div class="stagehead"><span class="t" id="sceneTitle">LiDAR · top-down</span></div>
      <canvas id="scene"></canvas>
      <img id="sceneImg" alt="" hidden>
      <button class="camtoggle hidden" id="camShow">▣ SHOW CAMERA</button>
      <div class="pip hidden" id="pip">
        <div class="ph"><span id="pipLabel">FRONT</span>
          <button id="pipSwap">SWAP</button><button id="pipHide">HIDE</button></div>
        <img id="pipImg" alt="camera feed">
      </div>
      <div class="stagefoot">
        <span class="m" id="mAge"><span class="lab">scan</span><b>—</b></span>
        <span class="m" id="mLat"><span class="lab">loop</span><b>—</b></span>
        <span class="m" id="mNear"><span class="lab">nearest</span><b>—</b></span>
        <span class="m" id="mScale"><span class="lab">rings</span><b>1 m</b></span>
        <span class="hint" id="sceneHint">drag the pad or hold W · A · S · D</span>
      </div>
    </main>

    <aside class="side">
      <section class="blk">
        <div class="blkhead"><h2>Telemetry</h2><span class="n" id="telRate">—</span></div>
        <div class="tiles" id="tiles"></div>
      </section>

      <section class="blk" id="ctlBlk">
        <div class="blkhead"><h2 id="ctlTitle">Manual drive</h2><span class="n" id="ctlArm">DISARMED</span></div>

        <div class="act" id="act"></div>

        <div class="gov">
          <span class="cap">speed cap <b id="capV">6</b>%</span>
          <button id="capD" aria-label="Lower speed cap">−</button>
          <button id="capU" aria-label="Raise speed cap">+</button>
        </div>

        <div class="pad">
          <button id="pF" style="grid-area:u" aria-label="Forward">▲</button>
          <button id="pL" style="grid-area:l" aria-label="Steer left">◄</button>
          <div class="c"><span>THR <b id="thrV">0</b>%</span><span>STR <b id="strV">+0.00</b></span></div>
          <button id="pR" style="grid-area:r" aria-label="Steer right">►</button>
          <button id="pB" style="grid-area:d" aria-label="Reverse">▼</button>
        </div>

        <div class="trim">
          <span class="lab">Steering trim</span>
          <input type="range" id="trim" min="-1" max="1" step="0.02" value="0" aria-label="Steering trim">
        </div>

        <div class="slider disabled" id="go" data-armed="0">
          <div class="fill" id="goFill"></div>
          <div class="txt" id="goTxt">SLIDE TO DRIVE ROUTE ››</div>
          <div class="knob" id="goKnob">›</div>
        </div>
        <button class="stopbtn" id="stopbtn" hidden>■ STOP FOLLOWING</button>

        <div id="detsWrap" hidden>
          <span class="lab">Detections · front</span>
          <div class="dets" id="dets"><div class="empty">enter PERCEPTION to run detection</div></div>
        </div>

        <div class="keys" id="keys">
          hold <kbd>W</kbd><kbd>S</kbd> drive · <kbd>A</kbd><kbd>D</kbd> steer, springs back ·
          <kbd>SPACE</kbd> E-STOP
        </div>
      </section>
    </aside>
  </div>

  <!-- ══ EVENTS ══ -->
  <footer class="log">
    <span class="lab">Events</span>
    <ul id="events"></ul>
    <span class="simtag" id="simtag" hidden>SIMULATED — NO CAR</span>
  </footer>
</div>

<script>
"use strict";
const $ = id => document.getElementById(id);
const clamp = (v,a,b) => v<a?a:v>b?b:v;
const fmt = (v,d=2) => v==null||Number.isNaN(v) ? "—" : v.toFixed(d);

/* ── theme: explicit control, not just prefers-color-scheme (§ dark themes) ── */
let hmi = "day";
try{ hmi = localStorage.getItem("hmi") || (matchMedia("(prefers-color-scheme: dark)").matches?"night":"day"); }catch(e){}
function applyTheme(){
  document.documentElement.dataset.hmi = hmi;
  $("themebtn").textContent = hmi==="night" ? "☀ DAY" : "☾ NIGHT";
  try{ localStorage.setItem("hmi", hmi); }catch(e){}
}
$("themebtn").onclick = () => { hmi = hmi==="night"?"day":"night"; applyTheme(); };
$("recbtn").onclick = () => {
  const on = !UI.rec;
  cmd(on ? "rec=1" : "rec=0");
  logEvent(on ? "recording started" : "recording stopped");
};
applyTheme();

/* ── local UI state ── */
const UI = {
  mode:"drive", autonomy:"MANUAL", cap:6,
  held:0, steerTarget:0, steerCur:0,
  estop:false, armed:false, following:false, live:false, rec:false
};
const EV = [];
function logEvent(text, kind){
  const d = new Date();
  EV.unshift({t:d.toTimeString().slice(0,8), text, kind:kind||""});
  if(EV.length>8) EV.pop();
  $("events").innerHTML = EV.map(e =>
    `<li class="${e.kind}"><time>${e.t}</time><span>${e.text}</span></li>`).join("");
}

/* ── transport: live endpoints, else simulate. Never fake being connected. ── */
async function cmd(q){
  if(!UI.live) return mockCmd(q);
  try{ await fetch("/cmd?"+q,{method:"POST"}); }catch(e){}
}
async function getState(){
  try{
    const r = await fetch("/state",{cache:"no-store"});
    if(!r.ok) throw 0;
    const j = await r.json();
    if(!UI.live){ UI.live = true; $("simtag").hidden = true; logEvent("link established"); }
    return j;
  }catch(e){
    if(UI.live){ UI.live = false; logEvent("link lost — telemetry simulated","alarm"); }
    $("simtag").hidden = false;
    return mockState();
  }
}

/* ── simulator: a plausible car on a stand, so the page opens in a working state ── */
const SIM = { t0:performance.now(), tach:0, estop:false, armed:false, follow:false };
function mockCmd(q){
  if(q.includes("estop=1")){ SIM.estop=true; SIM.armed=false; SIM.follow=false; }
  if(q.includes("clearstop=1")){ SIM.estop=false; SIM.armed=false; }
  if(q.includes("arm=off")) SIM.armed = false;
  else if(q.includes("arm=") && !SIM.estop) SIM.armed = true;
  if(q.includes("follow=1") && !SIM.estop){ SIM.follow=true; SIM.armed=true; }
  if(q.includes("follow=0")) SIM.follow=false;
}
function mockState(){
  const t = (performance.now()-SIM.t0)/1000;
  const moving = SIM.armed && (UI.held!==0 || SIM.follow);
  const speed = moving ? 0.34 + 0.05*Math.sin(t*1.7) : 0;
  SIM.tach += speed*0.05;
  return {
    mode: UI.mode,
    health:{ lidar:true, vesc:true, steer:true, cam:true,
             detector: UI.mode==="perception", loc: UI.mode!=="navigate" ? null : (t%23>17?false:true) },
    drive:{ armed:SIM.armed, estop:SIM.estop, duty: moving ? UI.cap*(UI.held||1) : 0 },
    tele:{ speed, v_in: 11.9 - 0.0009*t, temp_mos: 31.4 + 0.6*Math.sin(t/9),
           fault:"NONE", tach:SIM.tach },
    near: 0.55 + 0.42*Math.abs(Math.sin(t/5.5)),
    scan_age: 0.05 + 0.03*Math.abs(Math.sin(t*2)),
    loop_ms: 78 + 14*Math.sin(t*0.8),
    follow:{ on:SIM.follow, note: SIM.follow?"driving 1.84 m to goal":"" },
    heading: (t*7)%360,
    frames: Math.floor(t*3),
    front_dets: UI.mode==="perception"
      ? [{name:"person",conf:.71,vru:true},{name:"chair",conf:.58,vru:false},
         {name:"laptop",conf:.54,vru:false}] : [],
    _sim:true
  };
}

/* ── pre-arm: continuous while disarmed, first specific failure, prefixed ── */
function preArm(s){
  const out = [];
  if(s.drive && s.drive.estop) out.push("E-STOP latched — press CLEAR E-STOP to release");
  if(!s.health || !s.health.lidar) out.push("no LiDAR scans — is another process holding the port?");
  if(UI.mode==="navigate" && s.health && s.health.loc===false)
    out.push("localization not matched — park the car where mapping started");
  if(s.tele && s.tele.fault && s.tele.fault!=="NONE") out.push("VESC fault: "+s.tele.fault);
  if(s.tele && s.tele.v_in!=null && s.tele.v_in < 10.8) out.push("battery below arming minimum");
  return out;
}
let lastPreArm = "", lastPreArmAt = 0;
function renderPreArm(s){
  const f = preArm(s), el = $("prearm");
  if(!f.length){
    el.className = "prearm clear";
    $("prearmMsg").textContent = "all checks pass — ready to arm";
    $("prearmMore").textContent = "";
    lastPreArm = "";
    return;
  }
  el.className = "prearm";
  $("prearmMsg").textContent = "PreArm: " + f[0];
  $("prearmMore").textContent = f.length>1 ? "+"+(f.length-1)+" more" : "";
  const now = performance.now();
  if(f[0] !== lastPreArm || now - lastPreArmAt > 30000){   // re-announce every 30 s
    if(f[0] !== lastPreArm) logEvent("PreArm: "+f[0], "caution");
    lastPreArm = f[0]; lastPreArmAt = now;
  }
}

/* ── chips: glyph + text + border. Stale is dashed, not just dim. ── */
function chip(name, st, note){
  const g = st==="ok"?"●":st==="caution"?"▲":st==="alarm"?"■":"◌";
  const cls = st==="ok"?"":st;
  return `<span class="chip ${cls}"><span class="gl">${g}</span>${name}${note?" "+note:""}</span>`;
}
function renderChips(s){
  const h = s.health||{}, out = [];
  out.push(chip("LIDAR", h.lidar?"ok":"alarm", h.lidar?"":"NO DATA"));
  out.push(chip("VESC", h.vesc?"ok":"stale", h.vesc?"":"NO REPLY"));
  out.push(chip("STEER", h.steer?"ok":"stale"));
  out.push(chip("CAM", h.cam?"ok":"stale"));
  if(UI.mode==="navigate") out.push(chip("LOC", h.loc?"ok":"caution", h.loc?"":"NOT MATCHED"));
  if(UI.mode==="perception") out.push(chip("DETECT", h.detector?"ok":"stale"));
  $("chips").innerHTML = out.join("");
}

/* ── one top-level state word ── */
function renderState(s){
  const d = s.drive||{}, el = $("state");
  let w="READY", sub="disarmed · safe to approach", cls="", gl="●";
  if(d.estop){ w="E-STOP"; sub="latched — clear to release"; cls="alarm"; gl="■"; }
  else if(preArm(s).length){ w="NOT READY"; sub="see pre-arm below"; cls="caution"; gl="▲"; }
  else if(s.follow && s.follow.on){ w="DRIVING"; sub=s.follow.note||"following route"; cls=""; gl="▶"; }
  else if(d.armed){ w="ARMED"; sub="throttle live"; cls="caution"; gl="▲"; }
  el.className = "state "+cls;
  $("stateW").textContent = w; $("stateSub").textContent = sub; $("stateGl").textContent = gl;
  $("ctlArm").textContent = d.estop ? "E-STOP" : d.armed ? "ARMED" : "DISARMED";
}

/* ── telemetry tiles: every number carries an interpretation ── */
function tile(label, val, unit, mean, st){
  return `<div class="tile ${st||""}"><span class="lab">${label}</span>
    <div class="row"><span class="v">${val}</span><span class="u">${unit}</span></div>
    <div class="mean">${mean}</div></div>`;
}
function renderTiles(s){
  const t = s.tele||{}, near = s.near, sp = t.speed||0;
  const stopT = (sp>0.02 && near!=null) ? near/sp : null;
  const nearSt = near==null?"stale":near<0.5?"alarm":near<0.9?"caution":"";
  const battSt = t.v_in==null?"stale":t.v_in<10.8?"alarm":t.v_in<11.1?"caution":"";
  $("tiles").innerHTML =
    tile("Speed", fmt(sp,2), "m/s",
         sp<0.02 ? "stationary" : `${fmt(sp*3.6,1)} km/h over ground`) +
    tile("Clearance", fmt(near,2), "m",
         near==null ? "no LiDAR return" :
         stopT ? `${fmt(stopT,1)} s to contact at speed` : "path ahead clear", nearSt) +
    tile("Battery", fmt(t.v_in,1), "V",
         t.v_in==null ? "no VESC reply" :
         t.v_in<10.8 ? "below arming minimum" :
         `${fmt((t.v_in-9.9)/(12.6-9.9)*100,0)}% of 3S usable range`, battSt) +
    tile("MOS temp", fmt(t.temp_mos,1), "°C",
         t.temp_mos==null ? "no VESC reply" :
         t.temp_mos>70 ? "throttling risk" : "well inside limits");
}

/* ── scene: canvas radar. Free transparent, unknown hatched, categorical off-ramp. ── */
const cv = $("scene"), cx = cv.getContext("2d");
let SCAN = [];
function css(v){ return getComputedStyle(document.documentElement).getPropertyValue(v).trim(); }
function resize(){
  const r = cv.getBoundingClientRect(), dpr = Math.min(devicePixelRatio||1, 2);
  cv.width = Math.max(1, r.width*dpr); cv.height = Math.max(1, r.height*dpr);
  cx.setTransform(dpr,0,0,dpr,0,0);
}
new ResizeObserver(resize).observe(cv);

function drawScene(s){
  const w = cv.clientWidth, h = cv.clientHeight;
  if(!w||!h) return;
  cx.clearRect(0,0,w,h);
  const cxp = w/2, cyp = h/2, R = Math.min(w,h)/2 - 18, PPM = R/4.2;   // 4.2 m radius

  cx.strokeStyle = css("--struct"); cx.globalAlpha = .5; cx.lineWidth = 1;
  for(let m=1;m<=4;m++){ cx.beginPath(); cx.arc(cxp,cyp,m*PPM,0,7); cx.stroke(); }
  cx.beginPath(); cx.moveTo(cxp,cyp-R); cx.lineTo(cxp,cyp+R);
  cx.moveTo(cxp-R,cyp); cx.lineTo(cxp+R,cyp); cx.stroke();
  cx.globalAlpha = 1;

  // returns — neutral unless close enough to matter (display by exception)
  const stale = (s.scan_age||0) > 0.6;
  for(const p of SCAN){
    const px = cxp + p.y*PPM, py = cyp - p.x*PPM;
    const d = Math.hypot(p.x,p.y);
    cx.fillStyle = stale ? css("--struct") : d<0.5 ? css("--alarm") : d<0.9 ? css("--caution") : css("--val");
    cx.globalAlpha = stale ? .35 : 1;
    cx.fillRect(px-1.5, py-1.5, 3, 3);
  }
  cx.globalAlpha = 1;

  // footprint as a true polygon — clearance is the judgement being made
  const L = 0.28*PPM, W = 0.15*PPM;
  cx.strokeStyle = css("--t1"); cx.lineWidth = 1.5;
  cx.strokeRect(cxp-W, cyp-L, W*2, L*2);
  cx.beginPath(); cx.moveTo(cxp-W*0.6, cyp-L); cx.lineTo(cxp, cyp-L-7); cx.lineTo(cxp+W*0.6, cyp-L);
  cx.closePath(); cx.fillStyle = css("--t1"); cx.fill();

  if(stale){
    cx.fillStyle = css("--caution");
    cx.font = "600 11px 'IBM Plex Mono', monospace";
    cx.fillText("SCAN STALE", 14, h-14);
  }
}
function mockScan(t){
  const pts = [];
  for(let a=0;a<360;a+=2){
    if(a>168 && a<192) continue;                       // masked rear bumper
    const rad = a*Math.PI/180;
    let r = 2.6 + 1.1*Math.sin(rad*2 + t*0.15) + 0.5*Math.cos(rad*3);
    if(a>340 || a<20) r = Math.min(r, 0.75 + 0.35*Math.abs(Math.sin(t/5.5)));
    pts.push({ x:r*Math.cos(rad), y:r*Math.sin(rad) });
  }
  return pts;
}

/* ── commands ── */
function setAutonomy(a, cause){
  if(UI.autonomy===a) return;
  UI.autonomy = a;
  for(const [k,id] of [["MANUAL","m-manual"],["ASSISTED","m-assist"],["AUTO","m-auto"]])
    $(id).setAttribute("aria-pressed", String(k===a));
  logEvent(`autonomy → ${a}${cause?" — "+cause:""}`, a==="AUTO"?"caution":"");
}
["manual","assist","auto"].forEach((k,i) => {
  $("m-"+k).onclick = () => setAutonomy(["MANUAL","ASSISTED","AUTO"][i], "operator");
});

function setMode(m){
  UI.mode = m;
  for(const k of ["drive","map","navigate","perception"])
    $("t-"+k).setAttribute("aria-pressed", String(k===m));
  $("sceneTitle").textContent = { drive:"LiDAR · top-down", map:"SLAM · building map",
    navigate:"Navigation · saved map", perception:"Perception · front camera" }[m];
  $("ctlTitle").textContent = m==="navigate" ? "Route control" : "Manual drive";
  $("go").classList.remove("disabled");
  resetGo();
  $("sceneHint").textContent = m==="navigate" ? "click the map to set a goal"
    : m==="map" ? "drive slowly so scans overlap — then SAVE MAP"
    : "drag the pad or hold W · A · S · D";
  cmd("mode="+m);
  renderAct(); syncScene();
  logEvent("task → "+m.toUpperCase());
}
["drive","map","navigate","perception"].forEach(k => $("t-"+k).onclick = () => setMode(k));

/* the scene shows the map in MAP/NAVIGATE, the camera in PERCEPTION, radar otherwise */
function syncScene(){
  const m = UI.mode;
  const useImg = (m==="map" || m==="navigate" || m==="perception");
  $("scene").hidden = useImg;
  $("sceneImg").hidden = !useImg;
  if(m==="perception"){ $("sceneImg").src = UI.live ? "/cam/front.mjpg" : ""; }
  else if(useImg && !UI.live){ $("sceneImg").src = ""; }
  // camera PiP is useful while driving and mapping; in PERCEPTION the camera IS the scene
  const wantPip = (m==="drive" || m==="map" || m==="navigate");
  $("camShow").classList.toggle("hidden", !wantPip || UI.pipOn);
  $("pip").classList.toggle("hidden", !wantPip || !UI.pipOn);
  if(wantPip && UI.pipOn) setPipSrc();
}
UI.pipOn = false; UI.pipRear = false;
function setPipSrc(){
  $("pipLabel").textContent = UI.pipRear ? "REAR" : "FRONT";
  $("pipImg").src = UI.live ? (UI.pipRear ? "/cam/rear.mjpg?" : "/cam/front.mjpg?")+Date.now() : "";
}
$("camShow").onclick = () => { UI.pipOn = true; if(UI.pipRear) cmd("rear=1"); syncScene(); };
$("pipHide").onclick = () => { UI.pipOn = false; cmd("rear=0"); syncScene(); };
$("pipSwap").onclick = () => {
  UI.pipRear = !UI.pipRear; cmd("rear="+(UI.pipRear?1:0)); setPipSrc();
  logEvent("camera → "+(UI.pipRear?"rear":"front"));
};

/* contextual actions — what this task actually needs, nothing else */
function renderAct(){
  const a = $("act");
  if(UI.mode==="map"){
    a.innerHTML = `<button id="saveMap">\u{1F4BE} SAVE MAP</button>
      <span class="note" id="mapNote">drive slowly — overlapping scans make a clean map</span>`;
    $("saveMap").onclick = async () => {
      await cmd("save=1");
      $("mapNote").className = "note ok";
      $("mapNote").textContent = "save requested — confirm [map] saved in the console";
      logEvent("map save requested");
    };
  } else if(UI.mode==="navigate"){
    a.innerHTML = `<div class="seg2" role="group" aria-label="Environment">
        <button id="envIn" aria-pressed="true">INDOOR 7%</button>
        <button id="envOut" aria-pressed="false">OUTDOOR 9%</button></div>
      <span class="note" id="goalNote">click the map to set a goal</span>`;
    $("envIn").onclick = () => setEnv("indoor"); $("envOut").onclick = () => setEnv("outdoor");
  } else if(UI.mode==="perception"){
    a.innerHTML = `<span class="note">detector is resident only in this task — it is freed on exit</span>`;
  } else {
    a.innerHTML = `<span class="note">manual driving · LiDAR radar above</span>`;
  }
  $("detsWrap").hidden = UI.mode!=="perception";
}
function setEnv(e){
  cmd("env="+e);
  $("envIn").setAttribute("aria-pressed", String(e==="indoor"));
  $("envOut").setAttribute("aria-pressed", String(e==="outdoor"));
  logEvent("auto-drive duty → "+(e==="indoor"?"7%":"9%"));
}

/* spatial commands go into the scene, not a coordinate form */
$("sceneImg").addEventListener("click", e => {
  if(UI.mode!=="navigate") return;
  const r = e.target.getBoundingClientRect();
  const x = Math.round((e.clientX-r.left)/r.width*500), y = Math.round((e.clientY-r.top)/r.height*500);
  cmd("goalx="+x+"&goaly="+y);
  const n = $("goalNote"); if(n){ n.className="note ok"; n.textContent = "goal set — planning…"; }
  logEvent(`goal set at map (${x}, ${y})`);
});

$("estop").onclick = () => {
  setHeld(0); setSteer(0);
  cmd("estop=1");
  setAutonomy("MANUAL","E-STOP");
  logEvent("E-STOP latched","alarm");
};
$("clearstop").onclick = () => { cmd("clearstop=1"); logEvent("E-STOP cleared — still disarmed"); };

/* speed governor */
function setCap(v){ UI.cap = clamp(v,1,20); $("capV").textContent = UI.cap; }
$("capU").onclick = () => setCap(UI.cap+1);
$("capD").onclick = () => setCap(UI.cap-1);

/* throttle — streamed while held so the 0.5 s deadman stays fed */
let thrTimer = null;
function setHeld(d){
  if(d!==0 && UI.estop){
    $("keys").classList.add("warn");
    $("keys").innerHTML = "⛔ <b>E-STOP is latched</b> — press CLEAR E-STOP to release it.";
    return;
  }
  UI.held = d;
  $("pF").classList.toggle("on", d>0); $("pB").classList.toggle("on", d<0);
  if(!thrTimer) thrTimer = setInterval(() => {
    if(UI.held!==0){ cmd("throttle="+(UI.held*UI.cap/100).toFixed(3)); }
    else { cmd("throttle=0"); clearInterval(thrTimer); thrTimer = null; }
    $("thrV").textContent = Math.round(UI.held*UI.cap);
  }, 110);
}
/* steering springs back to the trim value, not to zero */
let steerTimer = null, trimVal = 0;
function applySteer(v){
  UI.steerCur = v; $("strV").textContent = (v<0?"":"+")+v.toFixed(2);
  cmd("steer="+v.toFixed(2));
}
function setSteer(t){
  UI.steerTarget = t;
  $("pL").classList.toggle("on", t < trimVal - 0.01);
  $("pR").classList.toggle("on", t > trimVal + 0.01);
  if(steerTimer) return;
  steerTimer = setInterval(() => {
    const d = UI.steerTarget - UI.steerCur;
    UI.steerCur += clamp(d, -0.22, 0.22);
    if(Math.abs(UI.steerCur-UI.steerTarget) < 0.03) UI.steerCur = UI.steerTarget;
    applySteer(UI.steerCur);
    if(UI.steerTarget===UI.steerCur){ clearInterval(steerTimer); steerTimer = null; }
  }, 60);
}
$("trim").oninput = e => { trimVal = parseFloat(e.target.value); UI.steerCur = trimVal; applySteer(trimVal); };

function hold(el, down, up){
  el.addEventListener("pointerdown", e => { e.preventDefault(); el.setPointerCapture(e.pointerId); down(); });
  el.addEventListener("pointerup", up); el.addEventListener("pointercancel", up);
  el.addEventListener("pointerleave", up);
}
hold($("pF"), () => setHeld(1),  () => setHeld(0));
hold($("pB"), () => setHeld(-1), () => setHeld(0));
hold($("pL"), () => setSteer(-1), () => setSteer(trimVal));
hold($("pR"), () => setSteer(1),  () => setSteer(trimVal));

/* slide-to-confirm — the one gesture for anything that moves the car */
function goLabel(){
  if(UI.mode==="navigate") return UI.following ? "FOLLOWING ROUTE" : "SLIDE TO DRIVE ROUTE \u203A\u203A";
  return UI.armed ? "DRIVING ENABLED" : "SLIDE TO ENABLE DRIVING \u203A\u203A";
}
function resetGo(){
  const sl = $("go");
  const done = UI.mode==="navigate" ? UI.following : UI.armed;
  sl.classList.toggle("done", done);
  $("goTxt").textContent = goLabel();
  $("stopbtn").hidden = !done;
  $("stopbtn").textContent = UI.mode==="navigate" ? "\u25A0 STOP FOLLOWING" : "\u25A0 DISARM";
  if(!done){ $("goKnob").style.transform = "translateX(0)"; $("goFill").style.width = "0"; }
}

(function(){
  const sl = $("go"), knob = $("goKnob"), fill = $("goFill");
  let dragging = false, startX = 0, x = 0, max = 0;
  function reset(){ x = 0; knob.style.transform = "translateX(0)"; fill.style.width = "0"; }
  sl.addEventListener("pointerdown", e => {
    if(sl.classList.contains("disabled")) return;
    if(UI.estop){ logEvent("refused — E-STOP is latched","caution"); return; }
    dragging = true; sl.dataset.armed = "1"; startX = e.clientX;
    max = sl.clientWidth - knob.offsetWidth - 6;
    sl.setPointerCapture(e.pointerId);
  });
  sl.addEventListener("pointermove", e => {
    if(!dragging) return;
    x = clamp(e.clientX - startX, 0, max);
    knob.style.transform = `translateX(${x}px)`;
    fill.style.width = (x + knob.offsetWidth) + "px";
  });
  function end(){
    if(!dragging) return;
    dragging = false; sl.dataset.armed = "0";
    if(x >= max - 4){
      if(UI.mode==="navigate"){
        cmd("follow=1"); UI.following = true;
        setAutonomy("AUTO","route accepted");
        logEvent("route accepted — auto-driving","caution");
      } else {
        cmd("arm=on"); UI.armed = true;
        setAutonomy("MANUAL","driving enabled");
        logEvent("throttle armed — manual driving","caution");
      }
      resetGo();
    } else reset();
  }
  sl.addEventListener("pointerup", end); sl.addEventListener("pointercancel", end);
  sl.addEventListener("keydown", e => {
    if(e.key===" " || e.key==="Enter"){ e.preventDefault(); x = max; end(); }
  });
  sl.tabIndex = 0;
  $("stopbtn").onclick = () => {
    if(UI.mode==="navigate"){
      cmd("follow=0"); UI.following = false;
      setAutonomy("MANUAL","operator stopped route");
      logEvent("route stopped");
    } else {
      cmd("arm=off"); UI.armed = false;
      logEvent("throttle disarmed");
    }
    reset(); resetGo();
  };
})();

/* keyboard */
const K = {};
addEventListener("keydown", e => {
  if(e.target.tagName==="INPUT") return;
  const k = e.key.toLowerCase();
  if([" ","arrowup","arrowdown","arrowleft","arrowright"].includes(k)) e.preventDefault();
  if(k===" "){ $("estop").click(); return; }
  if(K[k]) return; K[k] = true; recompute();
});
addEventListener("keyup", e => { const k = e.key.toLowerCase(); if(K[k]){ delete K[k]; recompute(); } });
function recompute(){
  setHeld((K.w||K.arrowup) ? 1 : (K.s||K.arrowdown) ? -1 : 0);
  setSteer((K.a||K.arrowleft) ? -1 : (K.d||K.arrowright) ? 1 : trimVal);
}

/* ── render loop: display rate decoupled from sample rate (2–5 Hz) ── */
let lastPaint = 0, lastState = null, fps = 0, frames = 0, fpsT = 0;
async function tick(){
  const s = await getState();
  lastState = s;
  UI.estop = !!(s.drive && s.drive.estop);
  UI.armed = !!(s.drive && s.drive.armed);
  $("estop").classList.toggle("latched", UI.estop);
  $("clearstop").hidden = !UI.estop;
  if(!UI.estop && $("keys").classList.contains("warn")){
    $("keys").classList.remove("warn");
    $("keys").innerHTML = "hold <kbd>W</kbd><kbd>S</kbd> drive · <kbd>A</kbd><kbd>D</kbd> steer, springs back · <kbd>SPACE</kbd> E-STOP";
  }
  const rc = s.rec || {};
  UI.rec = !!rc.on;
  $("recbtn").classList.toggle("reccing", UI.rec);
  $("recbtn").textContent = UI.rec
    ? `REC ${Math.round(rc.secs||0)}s · ${rc.scans||0} scans` : "● REC";
  if(UI.rec && rc.dropped) $("recbtn").title = `${rc.dropped} samples DROPPED`;

  UI.following = !!(s.follow && s.follow.on);
  resetGo();
  renderState(s); renderChips(s); renderPreArm(s); renderTiles(s);

  const age = s.scan_age;
  const ageEl = $("mAge"), latEl = $("mLat");
  ageEl.querySelector("b").textContent = age==null ? "—" : fmt(age,2)+" s";
  ageEl.classList.toggle("stale", age!=null && age>0.6);
  latEl.querySelector("b").textContent = s.loop_ms==null ? "—" : Math.round(s.loop_ms)+" ms";
  latEl.classList.toggle("stale", s.loop_ms>150);
  $("mNear").querySelector("b").textContent = s.near==null ? "—" : fmt(s.near,2)+" m";
  $("telRate").textContent = fps ? fps.toFixed(1)+" Hz" : "";

  if(UI.live && (UI.mode==="map" || UI.mode==="navigate"))
    $("sceneImg").src = "/map.jpg?"+Date.now();
  if(UI.mode==="map"){
    const n = $("mapNote");
    if(n && s.frames!=null) n.textContent = s.frames+" scans integrated · drive slowly, then SAVE MAP";
  }
  if(UI.mode==="perception"){
    const ds = s.front_dets||[];
    $("dets").innerHTML = ds.length
      ? ds.map(d => `<div class="d ${d.vru?"vru":""}"><b>${d.name}</b><span>${Math.round(d.conf*100)}%</span></div>`).join("")
      : '<div class="empty">no objects detected</div>';
  }
  const nav = s.follow||{};
  if(UI.mode==="navigate"){
    const n = $("goalNote");
    if(n && nav.note) { n.className = "note ok"; n.textContent = nav.note; }
  }
  setTimeout(tick, 140);        // control-relevant state under the 150 ms budget
}

function frame(ts){
  if(!fpsT) fpsT = ts;
  frames++;
  if(ts-fpsT > 1000){ fps = frames*1000/(ts-fpsT); frames = 0; fpsT = ts; }
  if(ts - lastPaint > 200){     // 5 Hz repaint — humans read 2–5 updates/s
    lastPaint = ts;
    if(!UI.live) SCAN = mockScan(ts/1000);
    if(lastState) drawScene(lastState);
  }
  requestAnimationFrame(frame);
}

resize();
logEvent("cockpit up · day mode");
renderAct(); setMode("drive");
tick();
requestAnimationFrame(frame);
</script>

</body></html>"""


# A browser that navigates away, refreshes, or swaps an <img> src drops the socket
# mid-response. That is normal, but ThreadingHTTPServer prints a full traceback for
# each one, and with several polled endpoints the console fills with them — which
# is exactly where a REAL error (a dead follow loop, a driver exception) would go
# unnoticed. Treat a client hang-up as the non-event it is.
CLIENT_GONE = (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)


class QuietServer(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        exc = sys.exc_info()[1]
        if isinstance(exc, CLIENT_GONE):
            return                      # client just went away; nothing to report
        super().handle_error(request, client_address)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        try:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except CLIENT_GONE:
            pass

    def _mjpeg(self, which):
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while HUB and HUB.running:
                jpg = HUB.front_jpg if which == "front" else HUB.rear_jpg
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode() +
                                     b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1 / 15.0)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            pass

    def _static(self, rel):
        """Serve the front end from disk.

        From disk rather than from a string in this file, on purpose: the CSS
        and the scene renderers are now big enough that they should be editable
        and reloadable without restarting the process that is holding the
        serial ports open.
        """
        rel = rel.lstrip("/")
        full = os.path.normpath(os.path.join(WEB_DIR, rel))
        # containment check FIRST — this server is reachable from the whole LAN
        if not full.startswith(WEB_DIR + os.sep) or not os.path.isfile(full):
            self.send_error(404); return
        ctype = {".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
                 ".js": "application/javascript; charset=utf-8", ".json": "application/json",
                 ".png": "image/png", ".jpg": "image/jpeg", ".svg": "image/svg+xml",
                 ".woff2": "font/woff2", ".map": "application/json"
                 }.get(os.path.splitext(full)[1].lower(), "application/octet-stream")
        try:
            with open(full, "rb") as fh:
                self._send(fh.read(), ctype)
        except OSError:
            self.send_error(404)

    def do_GET(self):
        p = urlparse(self.path).path
        if p == "/":
            index = os.path.join(WEB_DIR, "index.html")
            if os.path.isfile(index):
                self._static("index.html")
            else:
                # PAGE is the previous single-file cockpit, kept as a fallback
                # so a missing web/ directory degrades to a working screen
                # instead of a 404 on the machine you drive the car from.
                self._send(PAGE.encode(), "text/html; charset=utf-8")
        elif p.startswith("/web/"):
            self._static(p[5:])
        elif p == "/cam/front.mjpg":
            self._mjpeg("front")
        elif p == "/cam/rear.mjpg":
            self._mjpeg("rear")
        elif p == "/map.jpg":
            self._send(HUB.map_jpg or _placeholder("MAP"), "image/jpeg")
        elif p == "/scan":
            self._send(json.dumps({"pts": HUB.scan_points()}).encode(),
                       "application/json")
        elif p == "/state":
            with S_LOCK:
                self._send(json.dumps(STATE).encode(), "application/json")
        else:
            self.send_error(404)

    def do_POST(self):
        q = parse_qs(urlparse(self.path).query)

        if CMD_TOKEN and q.get("k", [""])[0] != CMD_TOKEN:
            # Refuse before anything is read out of the query string, so a
            # wrong token cannot set a mode, a goal, or a throttle on its way
            # to being rejected. Logged: a command you did not send arriving
            # from the network is something you want to know about.
            print(f"[auth] REFUSED command from {self.client_address[0]}", flush=True)
            self.send_error(403, "bad or missing token")
            return

        if "estop" in q:
            FOLLOW["on"] = False
            FOLLOW["note"] = "E-STOP"
            with C_LOCK:
                CTRL["estop"] = True; CTRL["armed"] = False; CTRL["throttle"] = 0.0
            print("[estop] LATCHED", flush=True)
        if "clearstop" in q:
            # The ONLY way out of a latched E-STOP. Deliberately its own command, and
            # deliberately leaves the car DISARMED: clearing the latch must not also
            # be a green light. Arming is a second, separate decision.
            with C_LOCK:
                CTRL["estop"] = False; CTRL["armed"] = False; CTRL["throttle"] = 0.0
            print("[estop] cleared by operator (still disarmed)", flush=True)
        if "mode" in q:
            HUB.set_mode(q["mode"][0])
        if "env" in q:
            HUB.env = "outdoor" if q["env"][0] == "outdoor" else "indoor"
        if "rear" in q:
            HUB.rear_on = (q["rear"][0] == "1")
        if "arm" in q:
            val = q["arm"][0]
            with C_LOCK:
                if CTRL["estop"] and val != "off":
                    # REFUSED. Arming used to clear the latch as a side effect, which
                    # meant the auto-arm on the first W keypress silently defeated the
                    # E-STOP you had just pressed. Clearing is now clearstop, only.
                    pass
                elif val == "on":
                    CTRL["armed"] = True
                elif val == "off":
                    CTRL["armed"] = False
                else:                       # toggle
                    CTRL["armed"] = not CTRL["armed"]
        if "throttle" in q:
            try:
                with C_LOCK:
                    CTRL["throttle"] = float(q["throttle"][0])
                    CTRL["last_cmd"] = time.monotonic()
            except ValueError:
                pass
        if "steer" in q:
            try:
                HUB.set_steer(float(q["steer"][0]))
            except ValueError:
                pass
        if "goal_fwd" in q and "goal_lat" in q:
            # The new scene is drawn in metres in the car frame, so it reports
            # clicks in metres. Convert here rather than teaching the browser
            # about map pixels, cell size and the map origin.
            try:
                fwd = float(q["goal_fwd"][0]); lat = float(q["goal_lat"][0])
                with S_LOCK:
                    pose = STATE.get("pose")
                if pose:
                    th = pose[2]
                    wx = pose[0] + fwd * math.cos(th) - lat * math.sin(th)
                    wy = pose[1] + fwd * math.sin(th) + lat * math.cos(th)
                    col, row = LidarSLAM.world_to_px(wx, wy, OUT)
                    HUB.set_goal(col, row)
            except (ValueError, AttributeError, TypeError) as e:
                print("[goal] metric goal refused:", e, flush=True)
        if "goalx" in q and "goaly" in q:
            try:
                HUB.set_goal(float(q["goalx"][0]), float(q["goaly"][0]))
            except ValueError:
                pass
        if "follow" in q:
            go = q["follow"][0] == "1"
            if go and PATH.get("cells") and HUB.mode == "navigate":
                with C_LOCK:
                    latched = CTRL["estop"]
                    if not latched:
                        CTRL["armed"] = True
                        CTRL["last_cmd"] = time.monotonic()
                if latched:
                    # GO also used to clear the latch. It no longer does.
                    FOLLOW["note"] = "refused: E-STOP is latched — clear it first"
                    print("[follow] REFUSED — E-STOP latched.", flush=True)
                else:
                    FOLLOW["arrived"] = False; FOLLOW["note"] = "starting"
                    FOLLOW["on"] = True
                    print("[follow] GO.", flush=True)
            else:
                FOLLOW["on"] = False
                with C_LOCK:
                    CTRL["throttle"] = 0.0
        if "rec" in q:
            if q["rec"][0] == "1":
                note = q.get("note", [""])[0]
                ok = HUB.start_recording(note=note)
                print(f"[rec] {'started' if ok else 'FAILED to start'}", flush=True)
            else:
                st = HUB.stop_recording()
                print(f"[rec] stopped: {st}", flush=True)
        if "save" in q:
            HUB.save_map()
        self._send(b"{}", "application/json")


def main():
    global HUB
    # Bind the port BEFORE opening a single serial device.
    #
    # The old order brought the whole Hub up first and bound afterwards, so a
    # port already in use produced the worst possible outcome: a second process
    # that had taken the VESC, the steering board, the LiDAR and the cameras
    # away from the instance actually serving the UI, then died. Two cockpits,
    # one set of hardware, and a screen showing neither. Fail before you touch
    # anything you would have to hand back.
    try:
        srv = QuietServer(("0.0.0.0", HTTP_PORT), H)
    except OSError as e:
        print(f"\ncannot bind port {HTTP_PORT}: {e}")
        print("something is already serving it — almost always an earlier cockpit.")
        print("  pkill -f cockpit.py && sleep 2")
        print("no hardware was opened, so nothing has been taken from it.")
        return 1

    HUB = Hub().start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping ...")
    finally:
        srv.shutdown()
        # server_close() is the one that actually releases the listening
        # socket. shutdown() only stops the accept loop, which is why a
        # restart could still find the port taken.
        srv.server_close()
        HUB.stop()
        print("stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
