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

HTTP_PORT = 8080
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
DUTY_INDOOR = 0.07
DUTY_OUTDOOR = 0.09

STATE = {}
S_LOCK = threading.Lock()
CTRL = {"armed": False, "estop": False, "throttle": 0.0, "steer": 0.0, "last_cmd": 0.0}
C_LOCK = threading.Lock()
GOAL = {"cell": None}
PATH = {"cells": None, "world": None}
FOLLOW = {"on": False, "arrived": False, "note": ""}

BLANK = None                    # placeholder jpeg


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
                self.last_scan = scan
                self.last_scan_t = time.monotonic()
                if self.mode == "map" and self.slam is not None:
                    self.slam.add_scan(scan)
                if self.mode == "navigate" and self.loc is not None:
                    self.loc.update(scan)
                    with S_LOCK:
                        STATE["pose"] = [round(v, 2) for v in self.loc.pose]
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
                    with S_LOCK:
                        STATE["front_dets"] = [
                            {"name": d["name"], "conf": round(d["conf"], 2), "vru": d["vru"]}
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
        while self.running:
            now = time.monotonic()
            with C_LOCK:
                estop = CTRL["estop"]; armed = CTRL["armed"]
                target = CTRL["throttle"]
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
            time.sleep(0.05)

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
            # scan is guaranteed fresh by the gate above. Both steer terms now share
            # config.STEER_SIGN, so blending them can no longer cancel out.
            nav = self.nav.plan(scan)
            blocked = nav["blocked"]; near = nav["nearest_ahead_m"]
            if not blocked and near < REACT_M:
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
            with S_LOCK:
                STATE["follow"] = {"on": FOLLOW["on"], "arrived": FOLLOW["arrived"],
                                   "note": FOLLOW["note"]}

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
            with S_LOCK:
                STATE["mode"] = self.mode
                STATE["env"] = self.env
                STATE["rear_on"] = self.rear_on
                STATE["tele"] = tele
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
            time.sleep(0.3)

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
<title>RoboCar — Mission Control</title>
<style>
 :root{--bg:#0a0e12;--panel:#0e141a;--edge:#17242c;--edge2:#22333d;
   --ac:#5fd3bc;--dim:#3f5563;--txt:#dfe8ee;--amber:#f0a020;--red:#ff5c5c;--ok:#39d98a}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--txt);
   font-family:ui-sans-serif,system-ui,"Segoe UI",sans-serif;
   background-image:linear-gradient(rgba(30,50,60,.05) 1px,transparent 1px),
     linear-gradient(90deg,rgba(30,50,60,.05) 1px,transparent 1px);
   background-size:26px 26px}
 .mono{font-family:ui-monospace,"SF Mono",Menlo,Consolas,monospace}
 header{display:flex;align-items:center;gap:16px;padding:9px 16px;
   border-bottom:1px solid var(--edge);background:#0b1116}
 .brand{font-weight:800;letter-spacing:3px;color:var(--txt);white-space:nowrap}
 .brand b{color:var(--ac)}
 .tabs{display:flex;gap:4px}
 .tab{font-size:.72rem;letter-spacing:2px;padding:7px 12px;border:1px solid var(--edge2);
   background:#0c141a;color:var(--dim);cursor:pointer;border-radius:3px;font-family:ui-monospace,monospace}
 .tab:hover{color:var(--ac);border-color:#2d4550}
 .tab.on{color:#031014;background:var(--ac);border-color:var(--ac);font-weight:700}
 .pills{display:flex;gap:6px;margin-left:auto;flex-wrap:wrap}
 .pill{font-size:.62rem;letter-spacing:1.5px;padding:4px 8px;border:1px solid var(--edge2);
   border-radius:3px;color:var(--dim);font-family:ui-monospace,monospace}
 .pill.on{color:var(--ok);border-color:#1f5a44;box-shadow:inset 0 0 8px rgba(57,217,138,.12)}
 .pill.warn{color:var(--amber);border-color:#5a4410}
 #estop{background:#7a1410;color:#fff;border:1px solid #b3261e;font-weight:800;
   letter-spacing:2px;padding:9px 16px;border-radius:4px;cursor:pointer}
 #estop.latched{animation:blink 1s steps(2) infinite}
 #clearstop{background:#3a2a0c;color:#ffe6b0;border:1px solid #7a5a12;font-weight:700;
   letter-spacing:1.5px;padding:9px 14px;border-radius:4px;cursor:pointer;
   font-family:ui-monospace,monospace;font-size:.72rem}
 #clearstop:hover{border-color:var(--amber)}
 @keyframes blink{50%{background:#b3261e}}
 .grid{display:grid;grid-template-columns:minmax(300px,1.05fr) minmax(320px,1.25fr) minmax(280px,1fr);
   gap:12px;padding:12px}
 @media(max-width:1100px){.grid{grid-template-columns:1fr}}
 .col{display:flex;flex-direction:column;gap:12px;min-width:0}
 .panel{background:var(--panel);border:1px solid var(--edge);border-radius:6px;
   display:flex;flex-direction:column;overflow:hidden}
 .phead{display:flex;align-items:center;justify-content:space-between;padding:7px 10px;
   border-bottom:1px solid var(--edge);background:#0b1218}
 .ptitle{font-size:.66rem;letter-spacing:2.5px;color:var(--ac)}
 .pbody{padding:10px;position:relative}
 .exp{background:none;border:1px solid var(--edge2);color:var(--dim);border-radius:3px;
   cursor:pointer;font-size:.7rem;padding:2px 7px;line-height:1}
 .exp:hover{color:var(--ac);border-color:#2d4550}
 .panel.full{position:fixed;inset:0;z-index:60;border-radius:0}
 .panel.full .pbody{flex:1;display:flex;align-items:center;justify-content:center;overflow:auto}
 .feed{display:block;width:100%;border-radius:3px;background:#05080b}
 .panel.full .feed,.panel.full canvas{width:auto;max-width:100%;max-height:100%}
 canvas{display:block;width:100%;background:#05080b;border-radius:3px}
 .map{cursor:crosshair}
 .btn{background:#12202a;color:var(--txt);border:1px solid var(--edge2);border-radius:4px;
   padding:8px 12px;font-size:.8rem;cursor:pointer;font-family:ui-monospace,monospace;letter-spacing:1px}
 .btn:hover{border-color:var(--ac)}
 .btn.go{background:#0f3a26;border-color:#2f7d57;color:#c9ffe6}
 .btn.warn{background:#3a2a0c;border-color:#7a5a12;color:#ffe6b0}
 .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
 .seg{display:flex;border:1px solid var(--edge2);border-radius:4px;overflow:hidden}
 .seg button{background:#0c141a;color:var(--dim);border:0;padding:7px 12px;cursor:pointer;
   font-family:ui-monospace,monospace;font-size:.72rem;letter-spacing:1px}
 .seg button.on{background:var(--ac);color:#03110d;font-weight:700}
 .lab{font-size:.6rem;letter-spacing:2px;color:var(--dim);margin-bottom:4px}
 .tele{display:grid;grid-template-columns:1fr 1fr;gap:8px}
 .cell{background:#0b1218;border:1px solid var(--edge);border-radius:4px;padding:8px}
 .cell .v{font-size:1.5rem;font-family:ui-monospace,monospace;color:var(--ac);line-height:1.1}
 .cell .u{font-size:.6rem;color:var(--dim);letter-spacing:1px}
 .bar{height:6px;background:#0a1218;border:1px solid var(--edge);border-radius:3px;margin-top:5px;overflow:hidden}
 .bar>i{display:block;height:100%;background:var(--ac)}
 input[type=range]{width:100%;accent-color:var(--ac)}
 .status{font-family:ui-monospace,monospace;font-size:.72rem;color:#9fb6c0;padding:6px 2px}
 .det{font-family:ui-monospace,monospace;font-size:.72rem;max-height:150px;overflow:auto}
 .det .d{display:flex;justify-content:space-between;padding:3px 6px;border-bottom:1px solid #101a20}
 .det .vru{color:var(--red)} .hide{display:none}
 .hint{font-size:.66rem;color:var(--dim);letter-spacing:.5px}
 .khint{font-size:.64rem;color:var(--dim);letter-spacing:.4px;margin-top:8px;
   border-top:1px solid var(--edge);padding-top:7px}
 .khint b{color:var(--ac);font-family:ui-monospace,monospace}
 .pad{display:grid;grid-template-columns:1fr 1.3fr 1fr;grid-template-rows:auto auto auto;
   grid-template-areas:". u ." "l c r" ". d .";gap:6px;margin-top:10px;align-items:stretch}
 .padbtn{font-size:1.2rem;padding:14px 0;border-radius:6px;cursor:pointer;user-select:none;
   background:#12202a;color:var(--txt);border:1px solid var(--edge2)}
 .padbtn:hover{border-color:var(--ac)}
 .padbtn.go{background:#0f3a26;border-color:#2f7d57;color:#c9ffe6}
 .padbtn.act{background:var(--ac);color:#03110d;border-color:var(--ac)}
 .padc{grid-area:c;display:flex;flex-direction:column;align-items:center;justify-content:center;
   font-family:ui-monospace,monospace;font-size:.72rem;color:var(--ac);
   background:#0a1218;border:1px solid var(--edge);border-radius:6px;line-height:1.5}
</style></head><body>
<header>
 <div class=brand>◆ ROBO<b>CAR</b> // MISSION CONTROL</div>
 <div class=tabs id=tabs>
   <div class=tab data-m=drive>DRIVE</div>
   <div class=tab data-m=map>MAP</div>
   <div class=tab data-m=navigate>NAVIGATE</div>
   <div class=tab data-m=perception>PERCEPTION</div>
 </div>
 <div class=pills id=pills>
   <span class=pill id=p_lidar>LIDAR</span>
   <span class=pill id=p_vesc>VESC</span>
   <span class=pill id=p_steer>STEER</span>
   <span class=pill id=p_cam>CAM</span>
   <span class=pill id=p_det>DET</span>
   <span class=pill id=p_loc>LOC</span>
 </div>
 <button id=clearstop class=hide>↺ CLEAR E-STOP</button>
 <button id=estop>■ E-STOP</button>
</header>

<div class=grid>
 <!-- LEFT: cameras + detections -->
 <div class=col>
   <div class=panel><div class=phead><span class=ptitle>FRONT CAMERA</span>
     <button class=exp data-t=pf>⤢</button></div>
     <div class=pbody id=pf><img class=feed id=frontcam src="/cam/front.mjpg" alt=front></div></div>
   <div class=panel><div class=phead><span class=ptitle>REAR CAMERA</span>
     <div class=row><button class=btn id=rearbtn style="padding:3px 8px;font-size:.66rem">ENABLE</button>
       <button class=exp data-t=pr>⤢</button></div></div>
     <div class=pbody id=pr><img class=feed id=rearcam alt=rear></div></div>
   <div class=panel id=detpanel><div class=phead><span class=ptitle>DETECTIONS</span></div>
     <div class=pbody><div class=det id=dets><div class=hint>enter PERCEPTION mode to run object detection</div></div></div></div>
 </div>

 <!-- CENTER: map + context controls -->
 <div class=col>
   <div class=panel><div class=phead><span class=ptitle id=maptitle>LIDAR RADAR</span>
     <button class=exp data-t=pm>⤢</button></div>
     <div class=pbody id=pm><img class="feed map" id=mapimg src="/map.jpg" alt=map></div></div>
   <div class=panel><div class=phead><span class=ptitle>MODE CONTROL</span></div>
     <div class=pbody>
       <div id=ctx_drive class=hint>Manual driving. Use the DRIVE panel on the right. Live LiDAR radar above.</div>
       <div id=ctx_map class=hide>
         <div class=row><button class="btn go" id=savemap>💾 SAVE MAP</button>
           <span class=status id=mapinfo>drive slowly — scans overlap = clean map</span></div>
         <div class=khint>Drive while it maps: hold <b>W</b>/<b>S</b> to move, <b>A</b>/<b>D</b> to steer,
           <b>SPACE</b> to stop. First press auto-enables driving. (Full pad on the right →)</div></div>
       <div id=ctx_nav class=hide>
         <div class=row><button class="btn go" id=gobtn>▶ GO (drive route)</button>
           <button class="btn warn" id=stopbtn>■ STOP</button></div>
         <div class=status id=navinfo>click a point on the map to set a destination</div></div>
       <div id=ctx_perc class=hide class=hint>Object detection is running on the front camera. Enable the rear camera to detect behind too.</div>
       <div class=status id=status></div>
     </div></div>
 </div>

 <!-- RIGHT: 3D view + telemetry + env + drive -->
 <div class=col>
   <div class=panel><div class=phead><span class=ptitle>3D LIDAR VIEW</span>
     <button class=exp data-t=p3>⤢</button></div>
     <div class=pbody id=p3><canvas id=view3d width=380 height=300></canvas></div></div>
   <div class=panel><div class=phead><span class=ptitle>TELEMETRY</span></div>
     <div class=pbody>
       <div class=tele>
         <div class=cell><div class=lab>SPEED</div><div class=v id=t_speed>0.00</div><div class=u>M/S</div></div>
         <div class=cell><div class=lab>DUTY</div><div class=v id=t_duty>0</div><div class=u>%</div>
           <div class=bar><i id=t_dutybar style=width:0%></i></div></div>
         <div class=cell><div class=lab>HEADING</div><div class=v id=t_head>—</div><div class=u>DEG</div></div>
         <div class=cell><div class=lab>BATTERY</div><div class=v id=t_volt>—</div><div class=u>V IN</div>
           <div class=bar><i id=t_voltbar style=width:0%></i></div></div>
         <div class=cell><div class=lab>MOS TEMP</div><div class=v id=t_tmos>—</div><div class=u>°C</div></div>
         <div class=cell><div class=lab>FAULT</div><div class=v id=t_fault style=font-size:.9rem;padding-top:8px>—</div></div>
       </div>
     </div></div>
   <div class=panel><div class=phead><span class=ptitle>ENVIRONMENT</span></div>
     <div class=pbody><div class=seg id=envseg>
       <button data-e=indoor class=on>INDOOR</button><button data-e=outdoor>OUTDOOR</button></div>
       <div class=hint style=margin-top:6px>sets auto-drive speed &amp; caution profile</div></div></div>
   <div class=panel><div class=phead><span class=ptitle>MANUAL DRIVE</span>
     <span class=pill id=armpill>DISARMED</span></div>
     <div class=pbody>
       <div class=row><button class=btn id=armbtn>⏻ ENABLE DRIVING</button>
         <span class=hint>speed <b id=lvl>6</b>%</span>
         <button class=btn id=lvlm style=padding:6px 10px>−</button>
         <button class=btn id=lvlp style=padding:6px 10px>+</button></div>
       <div class=pad>
         <button class="padbtn go" id=fwd style=grid-area:u>▲</button>
         <button class="padbtn" id=left style=grid-area:l>◄</button>
         <div class=padc id=padc><div id=thrLED>THR 0%</div><div id=steLED>STEER 0.0</div></div>
         <button class="padbtn" id=right style=grid-area:r>►</button>
         <button class="padbtn go" id=rev style=grid-area:d>▼</button></div>
       <div style=margin-top:8px><div class=lab>STEERING TRIM</div>
         <input type=range id=steer min=-1 max=1 step=0.02 value=0></div>
       <div class=khint id=khint>⌨ hold <b>W</b>/<b>S</b> drive · <b>A</b>/<b>D</b> steer (springs back) · <b>SPACE</b> = E-STOP</div>
     </div></div>
 </div>
</div>

<script>
let L=6, held=0, ka=null, mode='drive', rearOn=false;
async function cmd(q){try{await fetch('/cmd?'+q,{method:'POST'})}catch(e){}}
const $=id=>document.getElementById(id);

// ---- mode tabs ----
function setMode(m){mode=m;cmd('mode='+m);
  document.querySelectorAll('#tabs .tab').forEach(t=>t.classList.toggle('on',t.dataset.m===m));
  $('ctx_drive').className=(m==='drive')?'hint':'hide';
  $('ctx_map').className=(m==='map')?'':'hide';
  $('ctx_nav').className=(m==='navigate')?'':'hide';
  $('ctx_perc').className=(m==='perception')?'hint':'hide';
  $('maptitle').textContent=(m==='map')?'SLAM MAP (building)':(m==='navigate')?'NAVIGATION MAP':'LIDAR RADAR';
}
document.querySelectorAll('#tabs .tab').forEach(t=>t.addEventListener('click',()=>setMode(t.dataset.m)));

// ---- estop ----
let estopped=false;
$('estop').addEventListener('click',()=>{cmd('estop=1');setHeld(0);setSteerTarget(0);});
// Clearing is its own control, shown only while latched, and it leaves the car
// DISARMED — you still have to choose to drive afterwards.
$('clearstop').addEventListener('click',()=>{cmd('clearstop=1');});

// ---- expand panels ----
document.querySelectorAll('.exp').forEach(b=>b.addEventListener('click',()=>{
  $(b.dataset.t).closest('.panel').classList.toggle('full');}));

// ---- map click (navigate only) ----
$('mapimg').addEventListener('click',e=>{
  if(mode!=='navigate')return;
  const r=e.target.getBoundingClientRect();
  const x=(e.clientX-r.left)/r.width*500, y=(e.clientY-r.top)/r.height*500;
  $('navinfo').textContent='goal set ('+Math.round(x)+','+Math.round(y)+') — planning…';
  cmd('goalx='+Math.round(x)+'&goaly='+Math.round(y));});

// ---- navigate GO/STOP, map save ----
$('gobtn').addEventListener('click',()=>cmd('follow=1'));
$('stopbtn').addEventListener('click',()=>cmd('follow=0'));
$('savemap').addEventListener('click',()=>{cmd('save=1');$('mapinfo').textContent='map saved → maps/room';});

// ---- environment ----
document.querySelectorAll('#envseg button').forEach(b=>b.addEventListener('click',()=>{
  document.querySelectorAll('#envseg button').forEach(x=>x.classList.remove('on'));
  b.classList.add('on');cmd('env='+b.dataset.e);}));

// ---- rear cam ----
$('rearbtn').addEventListener('click',()=>{rearOn=!rearOn;cmd('rear='+(rearOn?1:0));
  $('rearbtn').textContent=rearOn?'DISABLE':'ENABLE';
  $('rearcam').src=rearOn?('/cam/rear.mjpg?'+Date.now()):'';});

// ---- manual drive (drive-pad + keyboard, auto-arm, spring-back steer) ----
let armed=false;
$('armbtn').addEventListener('click',()=>cmd('arm=toggle'));
$('lvlm').addEventListener('click',()=>{L=Math.max(1,L-1);$('lvl').textContent=L});
$('lvlp').addEventListener('click',()=>{L=Math.min(20,L+1);$('lvl').textContent=L});

// throttle: keep streaming while held (deadman), send one 0 on release
let thrLoop=null;
function setHeld(d){
  if(d!==0 && estopped){                            // latched: refuse, and say why
    $('khint').innerHTML='⛔ <b>E-STOP is latched</b> — press CLEAR E-STOP to release it.';
    return;}
  if(d!==0 && !armed){cmd('arm=on');armed=true;}   // auto-enable on first drive input
  held=d;
  $('fwd').classList.toggle('act',d>0); $('rev').classList.toggle('act',d<0);
  if(!thrLoop) thrLoop=setInterval(()=>{
    if(held!==0){cmd('throttle='+(held*L/100));}
    else{cmd('throttle=0');clearInterval(thrLoop);thrLoop=null;}
    $('thrLED').textContent='THR '+Math.round(held*L)+'%';
  },110);
}
// steering: momentary target that springs back to 0
let steerCur=0,steerTarget=0,steerLoop=null;
function applySteer(v){steerCur=v;$('steer').value=v.toFixed(2);
  $('steLED').textContent='STEER '+v.toFixed(1);cmd('steer='+v.toFixed(2));}
function startSteerLoop(){ if(steerLoop)return; steerLoop=setInterval(()=>{
  const dz=steerTarget-steerCur; steerCur+=Math.max(-0.22,Math.min(0.22,dz));
  if(Math.abs(steerCur-steerTarget)<0.03){steerCur=steerTarget;}
  applySteer(steerCur);
  if(steerTarget===0 && steerCur===0){clearInterval(steerLoop);steerLoop=null;}
},60);}
function setSteerTarget(t){steerTarget=t;
  $('left').classList.toggle('act',t<0); $('right').classList.toggle('act',t>0);
  startSteerLoop();}

// on-screen pad (mouse + touch, press-and-hold)
function bindHold(id,onDown,onUp){const b=$(id);
  b.addEventListener('mousedown',e=>{e.preventDefault();onDown();});
  b.addEventListener('mouseup',onUp);b.addEventListener('mouseleave',onUp);
  b.addEventListener('touchstart',e=>{e.preventDefault();onDown();});
  b.addEventListener('touchend',e=>{e.preventDefault();onUp();});}
bindHold('fwd',()=>setHeld(1),()=>setHeld(0));
bindHold('rev',()=>setHeld(-1),()=>setHeld(0));
bindHold('left',()=>setSteerTarget(-1),()=>setSteerTarget(0));
bindHold('right',()=>setSteerTarget(1),()=>setSteerTarget(0));
// steering trim slider stays where you leave it
$('steer').addEventListener('input',e=>{steerTarget=parseFloat(e.target.value);
  steerCur=steerTarget;applySteer(steerCur);});

// keyboard driving (works anywhere except when typing in a field)
const K={};
addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT')return;
  const k=e.key.toLowerCase();
  if([' ','arrowup','arrowdown','arrowleft','arrowright'].includes(k))e.preventDefault();
  if(k===' '){cmd('estop=1');setHeld(0);setSteerTarget(0);return;}
  if(K[k])return; K[k]=true; recompute();});
addEventListener('keyup',e=>{const k=e.key.toLowerCase();if(K[k]){delete K[k];recompute();}});
function recompute(){
  const f=(K['w']||K['arrowup'])?1:((K['s']||K['arrowdown'])?-1:0);
  const s=(K['a']||K['arrowleft'])?-1:((K['d']||K['arrowright'])?1:0);
  setHeld(f); setSteerTarget(s);}

// ---- map refresh (single-jpg endpoint) ----
setInterval(()=>{$('mapimg').src='/map.jpg?'+Date.now();},220);

// ---- 3D lidar view ----
const cv=$('view3d'), cx2=cv.getContext('2d');
function draw3d(pts){const W=cv.width,H=cv.height,cx=W/2,horizon=H*0.30,S=30;
  cx2.fillStyle='#05080b';cx2.fillRect(0,0,W,H);
  // ground grid (forward lines + lateral arcs)
  cx2.strokeStyle='rgba(40,70,80,.45)';cx2.lineWidth=1;
  for(let fx=1;fx<=5;fx++){const y=horizon+(6-fx)*((H-horizon)/6);
    cx2.beginPath();cx2.moveTo(20,y);cx2.lineTo(W-20,y);cx2.stroke();}
  for(let fy=-3;fy<=3;fy++){cx2.beginPath();
    cx2.moveTo(cx+fy*20,horizon);cx2.lineTo(cx+fy*S*2.2,H);cx2.stroke();}
  // points as vertical bars (near=warm, far=teal)
  for(const [fx,fy] of pts){if(fx<=0.05)continue;
    const depth=Math.min(fx,6), t=depth/6;
    const sx=cx+ (fy/ (0.5+depth*0.32))*S;
    const sy=horizon+(1-t)*(H-horizon);
    const barH=Math.max(3,26*(1-t));
    const g=Math.floor(210*t+60), r=Math.floor(230*(1-t)+50), b=Math.floor(190*t+70);
    cx2.strokeStyle='rgb('+r+','+g+','+b+')';cx2.lineWidth=2;
    cx2.beginPath();cx2.moveTo(sx,sy);cx2.lineTo(sx,sy-barH);cx2.stroke();}
  // car
  cx2.fillStyle='#5fd3bc';cx2.beginPath();cx2.moveTo(cx,H-6);
  cx2.lineTo(cx-7,H);cx2.lineTo(cx+7,H);cx2.closePath();cx2.fill();
  cx2.fillStyle='rgba(95,211,188,.6)';cx2.font='10px monospace';
  cx2.fillText('FWD',cx-11,horizon-6);}
async function poll3d(){try{const s=await(await fetch('/scan')).json();draw3d(s.pts||[]);}catch(e){}
  setTimeout(poll3d,200);}

// ---- state poll ----
function pill(id,on,warn){const e=$(id);e.className='pill'+(on?' on':(warn?' warn':''));}
async function poll(){try{const s=await(await fetch('/state')).json();
  const h=s.health||{};
  pill('p_lidar',h.lidar);pill('p_vesc',h.vesc);pill('p_steer',h.steer);pill('p_cam',h.cam);
  pill('p_det',h.detector,false);
  // LOC only means something on a saved map; amber = navigating with a pose we
  // do NOT trust, which is exactly when auto-drive refuses to move.
  pill('p_loc',h.loc,mode==='navigate' && !h.loc);
  const d=s.drive||{};
  armed=!!d.armed && !d.estop;                 // keep auto-arm flag in sync
  estopped=!!d.estop;
  $('estop').classList.toggle('latched',estopped);
  $('clearstop').className=estopped?'':'hide';
  if(!estopped && $('khint').textContent.indexOf('E-STOP is latched')>=0){
    $('khint').innerHTML='⌨ hold <b>W</b>/<b>S</b> drive · <b>A</b>/<b>D</b> steer (springs back) · <b>SPACE</b> = E-STOP';}
  $('armpill').textContent=d.estop?'E-STOP':(d.armed?'ARMED':'DISARMED');
  $('armpill').className='pill'+(d.armed&&!d.estop?' on':(d.estop?' warn':''));
  const t=s.tele||{};
  $('t_speed').textContent=(t.speed!=null?t.speed:0).toFixed(2);
  $('t_duty').textContent=(d.duty!=null?d.duty:0);
  $('t_dutybar').style.width=Math.min(100,Math.abs(d.duty||0)*5)+'%';
  $('t_head').textContent=(s.heading!=null?Math.round(s.heading):'—');
  $('t_volt').textContent=(t.v_in!=null?t.v_in:'—');
  $('t_voltbar').style.width=(t.v_in?Math.max(0,Math.min(100,(t.v_in-9)/(12.6-9)*100)):0)+'%';
  $('t_tmos').textContent=(t.temp_mos!=null?t.temp_mos:'—');
  const f=t.fault||'—';$('t_fault').textContent=f;$('t_fault').style.color=(f==='NONE'||f==='—')?'var(--ac)':'var(--red)';
  // navigate info
  const ff=s.follow||{};
  if(mode==='navigate'){$('navinfo').textContent=
    (s.goal?('goal '+s.goal.join(', ')+' · '+(s.reachable?('route '+s.path_len+' cells'):'NO ROUTE')):'click a point on the map to set a destination')
    +(ff.on?('  ▶ '+(ff.note||'driving')):(ff.arrived?'  ✔ arrived':''));}
  if(mode==='map'){$('mapinfo').textContent='frames: '+(s.frames||0)+'  ·  drive slowly, cover the room, SAVE MAP';}
  // detections
  if(mode==='perception'){const dv=$('dets');const ds=s.front_dets||[];
    dv.innerHTML=ds.length?ds.map(x=>'<div class="d'+(x.vru?' vru':'')+'"><span>'+x.name+'</span><span>'+Math.round(x.conf*100)+'%</span></div>').join(''):'<div class=hint>no objects detected</div>';}
 }catch(e){} setTimeout(poll,300);}

setMode('drive');poll();poll3d();
</script></body></html>"""


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

    def do_GET(self):
        p = urlparse(self.path).path
        if p == "/":
            self._send(PAGE.encode(), "text/html; charset=utf-8")
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
        if "save" in q:
            HUB.save_map()
        self._send(b"{}", "application/json")


def main():
    global HUB
    HUB = Hub().start()
    srv = QuietServer(("0.0.0.0", HTTP_PORT), H)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping ...")
    finally:
        srv.shutdown()
        HUB.stop()
        print("stopped.")


if __name__ == "__main__":
    sys.exit(main())
