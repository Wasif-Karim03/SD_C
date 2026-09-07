#!/usr/bin/env python3
"""
apps/navigate_web.py — Navigate mode, STAGE 2a: load the saved map, localize the
car on it, click a destination, and see the planned route. NO motors yet (this
step validates localization + planning before we let it drive).

  cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
  python3 navigate_web.py
  open http://<jetson-ip>:8080  -> CLICK on the map to set a goal

IMPORTANT: place the car where mapping STARTED (same spot/heading), so its pose
lines up with the map. Then push/drive it slowly and watch the blue car dot track
on the map; click a point and the green route appears (A*), replanned as it moves.

Lean (LiDAR only for localize + optional manual drive). Owns LiDAR/VESC/steering.
"""
import os
import sys
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
from drivers.lidar import RPLidarC1                       # noqa: E402
from perception.slam import LidarSLAM, RES, SIZE, ORIGIN  # noqa: E402
from perception import planner as P                       # noqa: E402
from perception.lidar_nav import LidarNavigator          # noqa: E402

HTTP_PORT = 8080
OUT = 500
MAP_PATH = os.path.join(ROOT, "maps", "room.npy")
MAXD = config.MAX_DUTY
RAMP = 0.01
DEADMAN_S = 0.4

# ---- Stage 2b: autonomous path-following (pure-pursuit) ------------------- #
LOOKAHEAD_M = 0.55        # aim at a point this far ahead along the route
GOAL_TOL_M = 0.25         # "arrived" when the car is within this of the goal
FOLLOW_DUTY = 0.07        # gentle forward duty while auto-driving (>= MIN_MOVE_DUTY)
STEER_GAIN = 1.8          # heading error (rad) -> steering command
FOLLOW_STEER_SIGN = config.STEER_SIGN  # ONE knob for both followers (config.py)
# NOTE: cockpit.py supersedes this app and carries the scan-staleness and
# localization-health gates. Prefer cockpit.py for any powered run.
FOLLOW_FORWARD_DEG = config.LIDAR_FORWARD_DEG  # car's forward in the map frame
REPLAN_EVERY_S = 1.0      # re-run A* this often as the car localizes/moves
REACT_M = 1.3             # swerve for unexpected obstacles within this range
BLOCK_GIVEUP_S = 6.0      # if blocked this long with no progress, stop following

MAP_PNG = [None]
STATE = {}
S_LOCK = threading.Lock()
CTRL = {"armed": False, "estop": False, "throttle": 0.0, "last_cmd": 0.0}
C_LOCK = threading.Lock()
GOAL = {"cell": None}                    # (row,col) or None
PATH = {"cells": None, "world": None}    # cells + world-point list for pursuit
FOLLOW = {"on": False, "arrived": False, "note": ""}


class Hub:
    def __init__(self):
        if not os.path.exists(MAP_PATH):
            raise SystemExit(f"no map at {MAP_PATH} — run mapper_web.py + SAVE MAP first.")
        self.grid = np.load(MAP_PATH)
        # trav  = observed-FREE cells: used for the green tint + snapping a click
        #         to a sensible destination.
        # blocked = ONLY inflated walls. A* is allowed to route through unknown
        #         (un-scanned) space too — the live camera+LiDAR brain handles
        #         whatever is actually there while driving. Requiring the whole
        #         route to be observed-free fragments the graph on a partial map
        #         and makes clicks silently produce no route.
        self.trav = P.traversable_mask(self.grid)     # for display + goal snapping
        self.blocked = P.obstacle_mask(self.grid)     # only walls block planning
        self.prob = 1.0 - 1.0 / (1.0 + np.exp(self.grid))
        self.loc = P.Localizer(self.grid)
        self.nav = LidarNavigator(react_m=REACT_M)   # live obstacle safety
        self.last_scan = None
        self.lidar = None
        self.vesc = self.steer = None
        self.v_lock = threading.Lock()
        self.running = False

    def start(self):
        print("lidar ...")
        self.lidar = RPLidarC1().connect()
        try:
            self.lidar.stop()
        except Exception:
            pass
        from drivers.vesc import VESC, resolve_port
        if os.path.exists(resolve_port()):
            self.vesc = VESC()
        try:
            from drivers.steering import ServoController
            self.steer = ServoController(); self.steer.center(read_reply=False)
        except Exception as e:  # noqa: BLE001
            print("  steering n/a:", e)
        self.running = True
        threading.Thread(target=self._loc_loop, daemon=True).start()
        threading.Thread(target=self._render_loop, daemon=True).start()
        threading.Thread(target=self._actuator_loop, daemon=True).start()
        threading.Thread(target=self._follow_loop, daemon=True).start()
        print("navigate ready. place car at the mapping start spot.")
        return self

    def _loc_loop(self):
        try:
            for scan in self.lidar.iter_scans(min_points=120):
                if not self.running:
                    break
                self.last_scan = scan            # shared with the follow loop
                self.loc.update(scan)
                with S_LOCK:
                    STATE["pose"] = [round(v, 2) for v in self.loc.pose]
        except Exception as e:  # noqa: BLE001
            print("loc loop ended:", e)

    def _render_loop(self):
        base = ((1.0 - self.prob) * 255).astype(np.uint8)
        base = np.flipud(base)
        base = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
        # tint the NAVIGABLE area green so you can see where clicks will work
        trav_img = cv2.resize((np.flipud(self.trav).astype(np.uint8) * 255),
                              (OUT, OUT), interpolation=cv2.INTER_NEAREST)
        base = cv2.resize(base, (OUT, OUT), interpolation=cv2.INTER_NEAREST)
        tint = trav_img > 0
        base[tint] = (0.6 * base[tint] + np.array([40, 120, 40])).clip(0, 255).astype(np.uint8)
        while self.running:
            img = base.copy()
            # planned path
            cells = PATH["cells"]
            if cells:
                poly = []
                for (r, c) in cells[::3]:
                    x, y = P.cell_to_world(r, c)
                    px, py = LidarSLAM.world_to_px(x, y, OUT)
                    poly.append([int(px), int(py)])
                if len(poly) > 1:
                    cv2.polylines(img, [np.array(poly, np.int32)], False, (80, 220, 80), 2)
            # goal
            if GOAL["cell"]:
                gx, gy = P.cell_to_world(*GOAL["cell"])
                px, py = LidarSLAM.world_to_px(gx, gy, OUT)
                cv2.drawMarker(img, (int(px), int(py)), (60, 60, 255),
                               cv2.MARKER_TILTED_CROSS, 16, 2)
            # car
            with S_LOCK:
                pose = STATE.get("pose", [0, 0, 0])
            px, py = LidarSLAM.world_to_px(pose[0], pose[1], OUT)
            cv2.circle(img, (int(px), int(py)), 6, (230, 150, 60), -1)
            ok, buf = cv2.imencode(".png", img)
            if ok:
                MAP_PNG[0] = buf.tobytes()
            time.sleep(0.2)

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
                STATE["drive"] = {"armed": armed, "estop": estop, "duty": round(duty * 100, 1)}
            time.sleep(0.05)

    def set_steer(self, v):
        if self.steer:
            try:
                self.steer.steer(max(-1.0, min(1.0, v)), read_reply=False)
            except Exception:
                pass

    def set_goal(self, col_px, row_px):
        # image px -> world -> cell
        wx, wy = LidarSLAM.px_to_world(col_px, row_px, OUT)
        goal_raw = P.world_to_cell(wx, wy)
        # snap the clicked goal to the nearest observed-free cell (a sensible target)
        goal = P.snap_to_traversable(self.trav, goal_raw)
        with S_LOCK:
            pose = STATE.get("pose", [0.0, 0.0, 0.0])
        start_raw = P.world_to_cell(pose[0], pose[1])
        # snap the car's start to the nearest cell that isn't a wall (may be unknown)
        start = P.snap_to_traversable(~self.blocked, start_raw)
        path = P.astar(self.blocked, start, goal) if (start and goal) else None
        print(f"[goal] click_px=({col_px:.0f},{row_px:.0f}) world=({wx:.2f},{wy:.2f}) "
              f"goal_raw={goal_raw} goal={goal} start_raw={start_raw} start={start} "
              f"path={'None' if path is None else str(len(path)) + ' cells'}", flush=True)
        GOAL["cell"] = goal
        PATH["cells"] = path
        PATH["world"] = [P.cell_to_world(r, c) for (r, c) in path] if path else None
        # a NEW goal cancels any in-progress auto-drive until you press GO again
        FOLLOW["on"] = False
        FOLLOW["arrived"] = False
        FOLLOW["note"] = ""
        with S_LOCK:
            STATE["goal"] = [round(wx, 2), round(wy, 2)]
            STATE["on_map"] = goal is not None
            STATE["path_len"] = len(path) if path else 0
            STATE["reachable"] = path is not None

    def _replan(self):
        """Re-run A* from the car's current cell to the goal; update PATH in place."""
        goal = GOAL["cell"]
        if not goal:
            return None
        with S_LOCK:
            pose = STATE.get("pose", [0.0, 0.0, 0.0])
        start = P.snap_to_traversable(~self.blocked, P.world_to_cell(pose[0], pose[1]))
        path = P.astar(self.blocked, start, goal) if start else None
        if path:
            PATH["cells"] = path
            PATH["world"] = [P.cell_to_world(r, c) for (r, c) in path]
        return path

    def _pursuit_target(self, x, y):
        """Pick the look-ahead point on the route; return (tx, ty, dist_to_goal)."""
        wp = PATH["world"]
        if not wp:
            return None
        # nearest route index to the car
        di = [(px - x) ** 2 + (py - y) ** 2 for (px, py) in wp]
        i = int(min(range(len(wp)), key=lambda k: di[k]))
        # walk forward along the route until we've gone LOOKAHEAD_M
        tx, ty = wp[-1]
        acc = 0.0
        for j in range(i, len(wp) - 1):
            ax, ay = wp[j]; bx, by = wp[j + 1]
            acc += math.hypot(bx - ax, by - ay)
            if acc >= LOOKAHEAD_M:
                tx, ty = bx, by
                break
        gx, gy = wp[-1]
        return tx, ty, math.hypot(gx - x, gy - y)

    def _follow_loop(self):
        last_plan = 0.0
        blocked_since = None
        while self.running:
            time.sleep(0.08)
            if not FOLLOW["on"]:
                blocked_since = None
                continue
            with C_LOCK:
                estop = CTRL["estop"]
            if estop:
                FOLLOW["on"] = False
                FOLLOW["note"] = "E-STOP"
                continue
            now = time.monotonic()
            if now - last_plan >= REPLAN_EVERY_S:
                last_plan = now
                self._replan()
            with S_LOCK:
                pose = STATE.get("pose", [0.0, 0.0, 0.0])
            x, y, th = pose[0], pose[1], pose[2]
            tgt = self._pursuit_target(x, y)
            if tgt is None:
                self._drive(0.0, 0.0); FOLLOW["note"] = "no route"; continue
            tx, ty, dgoal = tgt
            if dgoal < GOAL_TOL_M:
                self._drive(0.0, 0.0)
                FOLLOW["on"] = False; FOLLOW["arrived"] = True
                FOLLOW["note"] = "arrived"
                print("[follow] ARRIVED at goal.", flush=True)
                continue
            # pure-pursuit heading error (car's forward in the map frame)
            desired = math.atan2(ty - y, tx - x)
            car_heading = th + math.radians(FOLLOW_FORWARD_DEG)
            err = (desired - car_heading + math.pi) % (2 * math.pi) - math.pi
            steer = max(-1.0, min(1.0, FOLLOW_STEER_SIGN * STEER_GAIN * err))
            # live LiDAR safety
            scan = self.last_scan
            blocked = False; near = float("inf")
            if scan:
                nav = self.nav.plan(scan)
                blocked = nav["blocked"]; near = nav["nearest_ahead_m"]
                if not blocked and near < REACT_M:      # unexpected obstacle -> swerve
                    steer = max(-1.0, min(1.0, 0.5 * steer + 0.5 * nav["steer"]))
            if blocked:
                self._drive(0.0, steer)
                blocked_since = blocked_since or now
                FOLLOW["note"] = f"blocked {near:.2f}m — waiting"
                if now - blocked_since > BLOCK_GIVEUP_S:
                    self._drive(0.0, 0.0); FOLLOW["on"] = False
                    FOLLOW["note"] = "stopped: blocked too long"
                    print("[follow] blocked too long — stopping.", flush=True)
            else:
                blocked_since = None
                self._drive(FOLLOW_DUTY, steer)
                FOLLOW["note"] = f"driving {dgoal:.2f}m to goal"
            with S_LOCK:
                STATE["follow"] = {"on": FOLLOW["on"], "arrived": FOLLOW["arrived"],
                                   "note": FOLLOW["note"]}

    def _drive(self, throttle, steer):
        """Stream a throttle+steer command to the actuator loop (deadman-fed)."""
        with C_LOCK:
            if throttle != 0.0:
                CTRL["armed"] = True
            CTRL["throttle"] = throttle
            CTRL["last_cmd"] = time.monotonic()
        self.set_steer(steer)

    def stop(self):
        self.running = False
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
        if self.lidar:
            self.lidar.disconnect()


HUB = None
PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>RoboCar — Navigate</title>
<style>
 body{margin:0;background:#0a0e13;color:#e6edf3;font-family:system-ui,sans-serif}
 header{display:flex;align-items:center;justify-content:space-between;padding:10px 16px;border-bottom:1px solid #243140}
 button{background:#1a2530;color:#e6edf3;border:1px solid #32424f;border-radius:8px;padding:10px 14px;margin:3px;cursor:pointer}
 #estop{background:#b3261e;border:none;font-weight:800}
 .wrap{display:flex;flex-wrap:wrap;gap:16px;padding:16px}
 .card{background:#141c24;border:1px solid #243140;border-radius:12px;padding:12px}
 img{display:block;border-radius:8px;background:#000;width:min(90vw,560px);cursor:crosshair}
 .go{background:#123a24;border-color:#2f7d57}
 input[type=range]{width:100%}
 .pill{padding:2px 8px;border-radius:999px;font-size:.75rem;border:1px solid #243140}
</style></head><body>
<header><b>🧭 RoboCar — Navigate (click a goal · GO to auto-drive)</b>
 <div><span class=pill style="background:#123a24;border-color:#2f7d57">build follow-1</span>
   <span id=pill class=pill>—</span><button id=estop onclick=estop()>■ E-STOP</button></div></header>
<div class=wrap>
 <div class=card><h3 style=margin:.2em>Map — click to set destination</h3>
   <img id=map><div id=info style="color:#9fb0be;font-size:.8rem;margin-top:6px"></div>
   <div style="margin-top:8px">
     <button id=go class=go onclick=startfollow()>▶ GO (drive route)</button>
     <button onclick=stopfollow()>■ STOP following</button>
     <span id=fstat style="color:#9fb0be;font-size:.8rem;margin-left:8px"></span></div></div>
 <div class=card><h3 style=margin:.2em>Move car to test localization</h3>
   <button onclick=arm()>ARM / DISARM</button><br>
   <button class=go onmousedown="hold(1)" onmouseup=rel() onmouseleave=rel()>▲ FWD</button>
   <button class=go onmousedown="hold(-1)" onmouseup=rel() onmouseleave=rel()>▼ REV</button>
   <div style=margin:10px 0>level <b id=lvl>6</b>% <button onclick=lv(-1)>−</button><button onclick=lv(1)>+</button></div>
   <div>steering<br><input id=st type=range min=-1 max=1 step=0.02 value=0 oninput=steer(this.value)>
     <button onclick="document.getElementById('st').value=0;steer(0)">center</button></div>
   <p style="color:#9fb0be;font-size:.75rem">Place the car at the mapping START spot so it lines up.
    Push/drive slowly; the blue dot should track. Click a spot -> green route (A*).</p></div>
</div>
<script>
let L=6,held=0,ka=null;
async function cmd(q){try{await fetch('/cmd?'+q,{method:'POST'})}catch(e){}}
function arm(){cmd('arm=toggle')} function estop(){cmd('estop=1')}
function startfollow(){cmd('follow=1')} function stopfollow(){cmd('follow=0')}
function lv(d){L=Math.max(1,Math.min(20,L+d));document.getElementById('lvl').textContent=L}
function steer(v){cmd('steer='+v)}
function hold(d){held=d;s();ka=setInterval(s,150)} function rel(){held=0;clearInterval(ka);cmd('throttle=0')}
function s(){cmd('throttle='+(held*L/100))}
function pickGoal(e){const im=document.getElementById('map');const r=im.getBoundingClientRect();
 const x=(e.clientX-r.left)/r.width*500, y=(e.clientY-r.top)/r.height*500;
 console.log('pickGoal click ->', Math.round(x), Math.round(y));
 document.getElementById('info').textContent='clicked ('+Math.round(x)+','+Math.round(y)+') — planning…';
 cmd('goalx='+Math.round(x)+'&goaly='+Math.round(y));}
document.getElementById('map').addEventListener('click', pickGoal);
function refresh(){document.getElementById('map').src='/map.png?'+Date.now();}
async function poll(){try{const s=await(await fetch('/state')).json();const d=s.drive||{};
 document.getElementById('pill').textContent=d.estop?'E-STOP':(d.armed?'ARMED':'DISARMED');
 document.getElementById('info').textContent='pose: '+(s.pose?s.pose.join(', '):'-')+
   (s.goal?('  |  goal: '+s.goal.join(', ')+'  |  '+(s.reachable?('route '+s.path_len+' cells'):'NO ROUTE')):'  |  click a goal');
 const f=s.follow||{};
 document.getElementById('fstat').textContent=f.on?('▶ '+(f.note||'driving')):(f.arrived?'✔ arrived':(f.note||'idle'));
 }catch(e){} setTimeout(poll,300);}
setInterval(refresh,400);refresh();poll();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = urlparse(self.path).path
        if p == "/":
            self._send(PAGE.encode(), "text/html; charset=utf-8")
        elif p == "/map.png":
            if MAP_PNG[0]:
                self._send(MAP_PNG[0], "image/png")
            else:
                self.send_error(503)
        elif p == "/state":
            with S_LOCK:
                self._send(json.dumps(STATE).encode(), "application/json")
        else:
            self.send_error(404)

    def do_POST(self):
        q = parse_qs(urlparse(self.path).query)
        if "estop" in q:
            FOLLOW["on"] = False
            with C_LOCK:
                CTRL["estop"] = True; CTRL["armed"] = False; CTRL["throttle"] = 0.0
        if "follow" in q:
            go = q["follow"][0] == "1"
            if go and PATH.get("cells"):
                FOLLOW["arrived"] = False; FOLLOW["note"] = "starting"
                with C_LOCK:
                    CTRL["estop"] = False; CTRL["armed"] = True
                    CTRL["last_cmd"] = time.monotonic()
                FOLLOW["on"] = True
                print("[follow] GO — auto-driving the route.", flush=True)
            else:
                FOLLOW["on"] = False
                with C_LOCK:
                    CTRL["throttle"] = 0.0
                if not go:
                    print("[follow] STOP.", flush=True)
        if "arm" in q:
            with C_LOCK:
                CTRL["armed"] = not CTRL["armed"]
                if CTRL["armed"]:
                    CTRL["estop"] = False
        if "throttle" in q:
            try:
                with C_LOCK:
                    CTRL["throttle"] = float(q["throttle"][0]); CTRL["last_cmd"] = time.monotonic()
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
        self._send(b"{}", "application/json")


def main():
    global HUB
    HUB = Hub().start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), H)
    print(f"\nNavigate:  http://localhost:{HTTP_PORT}  (or http://<jetson-ip>:{HTTP_PORT})")
    print("Ctrl-C to stop.")
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
