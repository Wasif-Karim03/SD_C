#!/usr/bin/env python3
"""
apps/control_center.py — RoboCar mission control (single browser cockpit).

ONE page with everything:
  * Front camera + perception overlay (live)      * Rear camera (live)
  * LiDAR top-down map                             * 3D map (orbit)
  * Telemetry: speed, heading, odometry, VESC (duty/rpm/volts/temp/fault), GPS
  * Manual driving: ARM, hold-to-drive throttle (deadman), steering slider
  * BIG RED EMERGENCY STOP (latching)

This program OWNS the VESC + steering (only one process may), so the E-STOP truly
cuts the motor. Run this INSTEAD of drive.py / dashboard.py (they'd fight for the
ports). Nothing drives until you ARM, and a deadman cuts throttle if the browser
stops sending (tab hidden / network drop) within 0.4 s.

Run on the Jetson (cameras, LiDAR, VESC[battery on], steering, GPS all free):
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 control_center.py
Open  http://<jetson-ip>:8080   (hostname -I for the IP). Ctrl-C to stop.
"""
import os
import sys
import json
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                     # noqa: E402
from drivers.camera import Camera                 # noqa: E402
from drivers.lidar import ThreadedLidar           # noqa: E402
from perception.fusion import FusedNavigator      # noqa: E402
from control.odometry import Odometry             # noqa: E402

HTTP_PORT = 8080
MAXD = config.MAX_DUTY
RAMP = 0.01
DEADMAN_S = 0.4
SELFDRIVE_DUTY = 0.08        # gentle speed when the brain is driving (fwd/rev magnitude)
BRAIN_STALE_S = 0.6         # if the brain stops updating this long -> stop the car

STATE = {}
S_LOCK = threading.Lock()
FRONT_JPEG = [None]
REAR_JPEG = [None]

# ---- shared control flags (set from the browser) -------------------------- #
CTRL = {"armed": False, "estop": False, "throttle": 0.0, "steer": 0.0,
        "last_cmd": 0.0, "selfdrive": False,
        "brain_thr": 0.0, "brain_steer": 0.0, "brain_t": 0.0, "brain_mode": "-"}
C_LOCK = threading.Lock()


class Hub:
    def __init__(self):
        self.front = self.rear = self.lid = self.nav = self.brain = None
        self.rear_frame = None
        self.vesc = self.steer = self.compass = None
        self.v_lock = threading.Lock()
        self.odo = Odometry()
        self.running = False

    # ---- bring-up (each device optional) ---------------------------------- #
    def start(self):
        print("front camera ...")
        self.front = Camera("front").start()
        try:
            print("rear camera ...")
            self.rear = Camera("rear").start()
        except Exception as e:  # noqa: BLE001
            print("  rear cam n/a:", e)
        try:
            print("lidar ...")
            self.lid = ThreadedLidar().start()
        except Exception as e:  # noqa: BLE001
            print("  lidar n/a:", e)
        print("depth model + brain (YOLO on both cams) ...")
        from control.brain import SmartBrain
        self.brain = SmartBrain(device=0)
        self.nav = self.brain.front
        self.nav.estimate_camera(np.zeros((self.front.height, self.front.width, 3),
                                           np.uint8))
        from drivers.vesc import VESC, resolve_port
        if os.path.exists(resolve_port()):
            try:
                self.vesc = VESC()
                v = self.vesc.get_values()
                if v:
                    self.odo.update(v["tach"])
            except Exception as e:  # noqa: BLE001
                print("  vesc n/a:", e)
        else:
            print("  vesc n/a: port missing (motor battery off?)")
        try:
            from drivers.steering import ServoController
            self.steer = ServoController()
            self.steer.center(read_reply=False)
        except Exception as e:  # noqa: BLE001
            print("  steering n/a:", e)
        try:
            from drivers.compass import Compass
            self.compass = Compass().open()
        except Exception as e:  # noqa: BLE001
            print("  compass n/a:", e)

        self.running = True
        threading.Thread(target=self._perception_loop, daemon=True).start()
        if self.rear:
            threading.Thread(target=self._rear_loop, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()
        threading.Thread(target=self._actuator_loop, daemon=True).start()
        threading.Thread(target=self._gps_loop, daemon=True).start()
        print("hub running.")
        return self

    # ---- perception: front cam + fusion ---------------------------------- #
    def _perception_loop(self):
        seq = 0
        while self.running:
            frame, seq = self.front.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            frame = frame.copy()
            scan = None
            if self.lid:
                s, age = self.lid.latest()
                scan = s if (s and age < 0.5) else None
            d = self.brain.decide(frame, scan, self.rear_frame)
            fused = d["fused"]
            with C_LOCK:
                CTRL["brain_thr"] = d["throttle_sign"]
                CTRL["brain_steer"] = d["steer"]
                CTRL["brain_mode"] = d["mode"]
                CTRL["brain_t"] = time.monotonic()
            annotated = self.nav.cam_nav.draw(frame, fused["cam_depth"], fused["camera"])
            if self.brain.det:
                self.brain.det.draw(annotated, d["front_dets"])
            ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ok:
                FRONT_JPEG[0] = buf.tobytes()
            pts = []
            if scan:
                fwd = self.nav.lidar_nav.forward_deg
                for _q, a, dmm in scan:
                    dm = dmm / 1000.0
                    if dm > config.LIDAR_MIN_M:
                        pts.append([round(a, 1), round(dm, 2)])
            lid = fused["lidar"]
            with S_LOCK:
                STATE["fused"] = {
                    "blocked": bool(fused["blocked"]),
                    "steer": round(float(fused["steer"]), 2),
                    "nearest": (round(fused["nearest_ahead_m"], 2)
                                if fused["nearest_ahead_m"] is not None else None),
                    "source": fused["source"],
                    "cam_blocked": bool(fused["camera"]["blocked"]),
                    "cam_reach": round(float(fused["camera"]["center_reach"]), 2),
                    "lidar_blocked": (bool(lid["blocked"]) if lid else None),
                }
                STATE["lidar_points"] = pts
                STATE["forward_deg"] = self.nav.lidar_nav.forward_deg
                STATE["front_arc"] = self.nav.lidar_nav.front_arc
                STATE["brain"] = {"mode": d["mode"], "reason": d["reason"],
                                  "rear_near": d["rear_near"],
                                  "rear_blocked": d["rear_blocked"],
                                  "best_bearing": d["best_bearing"],
                                  "person_front": d["person_front"],
                                  "person_rear": d["person_rear"]}

    def _rear_loop(self):
        seq = 0
        while self.running:
            frame, seq = self.rear.read(wait=True, last_seq=seq)
            if frame is not None:
                self.rear_frame = frame
                ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok:
                    REAR_JPEG[0] = buf.tobytes()
            time.sleep(0.15)     # rear at ~6 fps to spare USB bandwidth

    # ---- telemetry: VESC speed + odometry + compass ---------------------- #
    def _telemetry_loop(self):
        last_tach, last_t = None, time.monotonic()
        while self.running:
            tel = {}
            if self.vesc:
                with self.v_lock:
                    v = None
                    try:
                        v = self.vesc.get_values()
                    except Exception:
                        pass
                if v:
                    now = time.monotonic()
                    speed = 0.0
                    if last_tach is not None and now > last_t:
                        speed = (v["tach"] - last_tach) * config.METERS_PER_TACH / (now - last_t)
                    last_tach, last_t = v["tach"], now
                    with C_LOCK:
                        st_norm = CTRL["steer"]
                    p = self.odo.update(v["tach"], steer_norm=st_norm)
                    tel = {"v_in": round(v["v_in"], 1), "duty": round(v["duty"] * 100, 1),
                           "erpm": v["erpm"], "motor_a": round(v["motor_current"], 1),
                           "temp": round(v["temp_mos"], 1), "fault": v["fault_name"],
                           "speed": round(speed, 2),
                           "odo": {"x": round(p["x"], 2), "y": round(p["y"], 2),
                                   "yaw": round(p["yaw_deg"], 0), "dist": round(p["dist"], 1)}}
            if self.compass:
                try:
                    c = self.compass.read()
                    tel["heading"] = round(c["heading_deg"], 0)
                except Exception:
                    pass
            with S_LOCK:
                STATE["tel"] = tel
            time.sleep(0.2)

    def _gps_loop(self):
        try:
            from drivers.gps import GPS
            gps = GPS().open()
        except Exception as e:  # noqa: BLE001
            with S_LOCK:
                STATE["gps"] = {"err": str(e)}
            return
        while self.running:
            try:
                s = gps.poll(2.0)
                with S_LOCK:
                    STATE["gps"] = {"fix": s["fix_quality"], "sats": s["sats_used"],
                                    "lat": (round(s["lat"], 6) if s["lat"] else None),
                                    "lon": (round(s["lon"], 6) if s["lon"] else None)}
            except Exception:
                time.sleep(1.0)

    # ---- actuator: throttle (deadman) + estop ---------------------------- #
    def _actuator_loop(self):
        duty = 0.0
        while self.running:
            now = time.monotonic()
            with C_LOCK:
                estop = CTRL["estop"]
                armed = CTRL["armed"]
                sd = CTRL["selfdrive"]
                man_thr = CTRL["throttle"]
                man_fresh = (now - CTRL["last_cmd"]) < DEADMAN_S
                b_thr = CTRL["brain_thr"]
                b_steer = CTRL["brain_steer"]
                b_fresh = (now - CTRL["brain_t"]) < BRAIN_STALE_S
                b_mode = CTRL["brain_mode"]
            steer_cmd = None
            if estop:
                target = 0.0
            elif sd:                                  # SELF-DRIVE: the brain drives
                if b_fresh:
                    target = b_thr * SELFDRIVE_DUTY    # +fwd / -rev / 0 hold
                    steer_cmd = b_steer
                else:
                    target = 0.0                       # brain stale -> stop
            elif armed and man_fresh:                  # MANUAL deadman
                target = man_thr
            else:
                target = 0.0
            target = max(-MAXD, min(MAXD, target))
            if duty < target:
                duty = min(target, duty + RAMP)
            else:
                duty = max(target, duty - RAMP)
            if steer_cmd is not None:                  # brain steering (outside C_LOCK)
                self.set_steer(steer_cmd)
            if self.vesc:
                with self.v_lock:
                    try:
                        self.vesc.set_duty(duty)
                    except Exception:
                        pass
            with S_LOCK:
                STATE["drive"] = {"armed": armed, "estop": estop, "selfdrive": sd,
                                  "mode": (b_mode if sd else
                                           ("manual" if armed else "idle")),
                                  "duty_cmd": round(duty * 100, 1)}
            time.sleep(0.05)

    def set_steer(self, norm):
        norm = max(-1.0, min(1.0, norm))
        with C_LOCK:
            CTRL["steer"] = norm
        if self.steer:
            try:
                self.steer.steer(norm, read_reply=False)
            except Exception:
                pass

    def build_cloud(self):
        """On-demand 3D cloud from current front+rear frames (reuses open cams)."""
        STRIDE, DMIN, DMAX, FX, CAM_H = 7, 0.25, 8.0, 554.0, 0.15
        out = []

        def cam_cloud(cam, rear):
            if cam is None:
                return
            frame, _ = cam.read(wait=True)
            if frame is None:
                return
            depth = self.nav.cam_nav.estimate(frame)
            h, w = depth.shape
            cx, cy = w / 2.0, h / 2.0
            for vv in range(0, h, STRIDE):
                for uu in range(0, w, STRIDE):
                    d = float(depth[vv, uu])
                    if d < DMIN or d > DMAX:
                        continue
                    X = (uu - cx) / FX * d
                    Y = (vv - cy) / FX * d
                    Z = d
                    if not rear:
                        cxm, cym, czm = Z + 0.1, -X, -Y + CAM_H
                    else:
                        cxm, cym, czm = -(Z + 0.1), X, -Y + CAM_H
                    b, g, r = frame[vv, uu]
                    out.append([round(cxm, 3), round(cym, 3), round(czm, 3),
                                min(255, int(r * 2.2)), min(255, int(g * 2.2)),
                                min(255, int(b * 2.2))])
        cam_cloud(self.front, False)
        cam_cloud(self.rear, True)
        n_cam = len(out)
        if self.lid:
            s, age = self.lid.latest()
            if s and age < 0.5:
                fwd = self.nav.lidar_nav.forward_deg
                for _q, a, dmm in s:
                    d = dmm / 1000.0
                    if config.LIDAR_MIN_M < d < DMAX:
                        rel = np.radians(a - fwd)
                        out.append([round(float(d * np.cos(rel)), 3),
                                    round(float(d * np.sin(rel)), 3), 0.08,
                                    240, 155, 74])
        return {"points": out, "n_cam": n_cam, "n_lidar": len(out) - n_cam}

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
        if self.lid:
            self.lid.stop()
        if self.front:
            self.front.release()
        if self.rear:
            self.rear.release()


HUB = None
PAGE = r"""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>RoboCar — Control Center</title>
<style>
 :root{--bg:#0a0e13;--card:#141c24;--line:#243140;--ink:#e6edf3;--mut:#9fb0be;--acc:#5ab0da}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--ink);font-family:system-ui,Segoe UI,sans-serif}
 header{display:flex;align-items:center;justify-content:space-between;padding:10px 16px;
   border-bottom:1px solid var(--line)}
 header b{font-size:1.05rem}
 #estop{background:#b3261e;color:#fff;border:none;border-radius:10px;padding:14px 22px;
   font-size:1.1rem;font-weight:800;cursor:pointer;letter-spacing:.03em}
 #estop.armedstop{animation:none}
 #estop:hover{background:#d0392f}
 #sdbtn{background:#1a2530;color:#fff;border:1px solid #32424f;border-radius:10px;
   padding:14px 18px;font-size:1rem;font-weight:700;cursor:pointer;margin-right:8px}
 #sdbtn.on{background:#123a24;border-color:#2f7d57}
 #sdbtn:hover{border-color:#5ab0da}
 .grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;padding:12px}
 @media(max-width:1000px){.grid{grid-template-columns:1fr}}
 .card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:10px}
 .card h2{margin:0 0 8px;font-size:.72rem;letter-spacing:.08em;text-transform:uppercase;color:#e39b4a}
 img,canvas{width:100%;border-radius:8px;background:#000;display:block}
 .tel{display:grid;grid-template-columns:1fr 1fr;gap:8px 14px;font-size:.9rem}
 .tel div span{color:var(--mut);font-size:.72rem;display:block}
 .tel b{font-size:1.15rem}
 .big{font-size:1.5rem;font-weight:800}.ok{color:#4fbf8f}.blk{color:#e8735a}
 .controls button{background:#1a2530;color:var(--ink);border:1px solid var(--line);
   border-radius:8px;padding:10px 14px;font-size:.95rem;cursor:pointer;margin:3px}
 .controls button:hover{border-color:var(--acc)}
 .go{background:#123a24;border-color:#2f7d57}
 input[type=range]{width:100%}
 .pill{padding:2px 8px;border-radius:999px;font-size:.75rem;border:1px solid var(--line)}
 .exp{background:#1a2530;color:var(--ink);border:1px solid var(--line);border-radius:6px;
   cursor:pointer;font-size:.85rem;line-height:1;padding:2px 7px;float:right;margin-left:6px}
 .exp:hover{border-color:var(--acc)}
 .clickable{cursor:zoom-in}
 .card.zoom{position:fixed;inset:6px;z-index:200;margin:0;overflow:auto;
   box-shadow:0 0 0 9999px rgba(0,0,0,.75)}
 .card.zoom img,.card.zoom canvas{max-height:86vh;max-width:100%;width:auto;margin:6px auto}
 .card.zoom .tel{font-size:1.6rem}
 #closez{position:fixed;top:12px;right:14px;z-index:210;display:none;background:#b3261e;
   color:#fff;border:none;border-radius:8px;padding:8px 16px;cursor:pointer;font-weight:700}
 #closez.show{display:block}
</style></head><body>
<header>
 <b>🚗 RoboCar — Control Center</b>
 <div><span id=armpill class=pill>DISARMED</span>
   <button id=sdbtn onclick=selfdrive()>▶ SELF-DRIVE</button>
   <button id=estop onclick=estop()>■ EMERGENCY STOP</button></div>
</header>
<button id=closez onclick=closeZoom()>✕ Close (Esc)</button>
<div class=grid>
 <div class=card id=cardFront><h2>Front camera + perception
   <button class=exp onclick="zoom('cardFront')">⤢</button></h2>
   <img id=front class=clickable onclick="zoom('cardFront')"></div>
 <div class=card id=cardRear><h2>Rear camera
   <button class=exp onclick="zoom('cardRear')">⤢</button></h2>
   <img id=rear class=clickable onclick="zoom('cardRear')"></div>
 <div class=card id=cardLidar><h2>LiDAR map (top-down)
   <button class=exp onclick="zoom('cardLidar')">⤢</button></h2>
   <canvas id=lidar width=360 height=360 class=clickable onclick="zoom('cardLidar')"></canvas></div>

 <div class=card id=cardCloud><h2>3D map
   <button class=exp onclick="zoom('cardCloud')">⤢</button>
   <button style="float:right;font-size:.7rem" onclick=cloud()>recapture</button></h2>
   <canvas id=cloud3d width=360 height=300></canvas></div>

 <div class=card id=cardTel><h2>Telemetry
   <button class=exp onclick="zoom('cardTel')">⤢</button></h2>
  <div style=text-align:center;margin-bottom:8px>
    <div id=state class=big>—</div><div id=src style="color:var(--mut);font-size:.72rem"></div>
  </div>
  <div class=tel>
   <div><span>SPEED</span><b id=speed>—</b> m/s</div>
   <div><span>NEAREST AHEAD</span><b id=near>—</b></div>
   <div><span>HEADING</span><b id=head>—</b>°</div>
   <div><span>ODO YAW</span><b id=yaw>—</b>°</div>
   <div><span>DUTY</span><b id=duty>—</b>%</div>
   <div><span>MOTOR eRPM</span><b id=erpm>—</b></div>
   <div><span>BATTERY</span><b id=vin>—</b> V</div>
   <div><span>FET TEMP</span><b id=temp>—</b>°C</div>
   <div><span>ODO DIST</span><b id=dist>—</b> m</div>
   <div><span>FAULT</span><b id=fault>—</b></div>
   <div><span>GPS</span><b id=gps>—</b></div>
   <div><span>DRIVE DUTY</span><b id=dduty>—</b>%</div>
  </div>
 </div>

 <div class=card id=cardBrain><h2>Self-drive brain
   <button class=exp onclick="zoom('cardBrain')">⤢</button></h2>
   <div style=text-align:center>
     <div id=bmode class=big>—</div>
     <div id=breason style="color:var(--mut);font-size:.82rem;min-height:2.6em"></div></div>
   <div class=tel style=margin-top:6px>
     <div><span>REAR</span><b id=brear>—</b></div>
     <div><span>OPEN DIRECTION</span><b id=bopen>—</b>°</div>
     <div><span>PERSON FRONT</span><b id=bpf>—</b></div>
     <div><span>PERSON REAR</span><b id=bpr>—</b></div>
   </div>
   <p style="color:var(--mut);font-size:.72rem">DRIVE = clear · RECOVER = back out
    toward open · HOLD = boxed in / person close</p></div>

 <div class=card controls id=cardCtrl><h2>Manual control
   <button class=exp onclick="zoom('cardCtrl')">⤢</button></h2>
  <div class=controls>
   <button id=armbtn onclick=toggleArm()>ARM</button>
   <button class=go onmousedown="hold(1)" onmouseup="rel()" onmouseleave="rel()"
     ontouchstart="hold(1)" ontouchend="rel()">▲ HOLD FWD</button>
   <button class=go onmousedown="hold(-1)" onmouseup="rel()" onmouseleave="rel()"
     ontouchstart="hold(-1)" ontouchend="rel()">▼ HOLD REV</button>
  </div>
  <div style=margin:10px 0>throttle level: <b id=lvl>6</b>%
   <button onclick="lvladj(-1)">−</button><button onclick="lvladj(1)">+</button></div>
  <div>steering<br><input id=steer type=range min=-1 max=1 step=0.02 value=0
     oninput="steerCmd(this.value)" onchange="steerCmd(this.value)">
   <button onclick="document.getElementById('steer').value=0;steerCmd(0)">center</button></div>
  <p style="color:var(--mut);font-size:.75rem">Hold a drive button to move (deadman —
   releasing or losing the page stops it). ARM first. E-STOP latches until cleared.</p>
 </div>
</div>
<script>
let lvl=6, held=0, ka=null;
async function cmd(q){try{await fetch('/cmd?'+q,{method:'POST'})}catch(e){}}
function toggleArm(){cmd('arm=toggle')}
function estop(){cmd('estop=1')}
function selfdrive(){cmd('selfdrive=toggle')}
function lvladj(d){lvl=Math.max(1,Math.min(20,lvl+d));document.getElementById('lvl').textContent=lvl}
function steerCmd(v){cmd('steer='+v)}
function hold(dir){held=dir;send();ka=setInterval(send,150)}
function rel(){held=0;clearInterval(ka);cmd('throttle=0')}
function send(){cmd('throttle='+(held*lvl/100))}
// lidar canvas
const lc=document.getElementById('lidar'),lx=lc.getContext('2d');
function drawLidar(s){lx.fillStyle='#0b0f14';lx.fillRect(0,0,lc.width,lc.height);
 const CX=lc.width/2,CY=lc.height/2,MR=5,SC=(lc.width/2-10)/MR;
 lx.strokeStyle='#22303c';for(let r=1;r<=MR;r++){lx.beginPath();lx.arc(CX,CY,r*SC,0,7);lx.stroke();}
 const fwd=s.forward_deg||0,fa=(s.front_arc||60);
 (s.lidar_points||[]).forEach(p=>{let rel=(p[0]-fwd)*Math.PI/180,d=p[1];
   let x=CX+d*SC*Math.sin(rel),y=CY-d*SC*Math.cos(rel);
   let inf=Math.abs(((p[0]-fwd+540)%360)-180)<=fa/2;
   lx.fillStyle=(inf&&d<0.6)?'#e8735a':'#4fbf8f';lx.fillRect(x-1,y-1,2,2);});
 lx.fillStyle='#5ab0da';lx.beginPath();lx.moveTo(CX,CY-7);lx.lineTo(CX-5,CY+6);
 lx.lineTo(CX+5,CY+6);lx.fill();}
// 3D canvas
const c3=document.getElementById('cloud3d'),c3x=c3.getContext('2d');
let cpts=[],az=0.7,el=-0.5,sc=34,drag=0,px,py;
c3.addEventListener('mousedown',e=>{drag=1;px=e.clientX;py=e.clientY});
addEventListener('mouseup',()=>drag=0);
addEventListener('mousemove',e=>{if(!drag)return;az-=(e.clientX-px)*0.01;
 el+=(e.clientY-py)*0.01;px=e.clientX;py=e.clientY;draw3d()});
function pr(x,y,z){const ca=Math.cos(az),sa=Math.sin(az),x1=x*ca-y*sa,y1=x*sa+y*ca,
 ce=Math.cos(el),se=Math.sin(el),z2=y1*se+z*ce;return [c3.width/2+x1*sc,c3.height/2-z2*sc];}
function draw3d(){c3x.fillStyle='#0b0f14';c3x.fillRect(0,0,c3.width,c3.height);
 for(let i=0;i<cpts.length;i++){const p=cpts[i],s=pr(p[0],p[1],p[2]);
   c3x.fillStyle='rgb('+p[3]+','+p[4]+','+p[5]+')';c3x.fillRect(s[0],s[1],2,2);}
 const O=pr(0,0,0);c3x.fillStyle='#5ab0da';c3x.fillRect(O[0]-3,O[1]-3,6,6);}
async function cloud(){try{const d=await(await fetch('/cloud')).json();
 const st=Math.max(1,Math.floor(d.points.length/6000));
 cpts=d.points.filter((_,i)=>i%st===0);draw3d();}catch(e){}}
// zoom any panel to full screen
let zoomed=null,orig={};
function zoom(id){const card=document.getElementById(id);
 if(zoomed===card){closeZoom();return;} if(zoomed)closeZoom();
 zoomed=card;card.classList.add('zoom');document.getElementById('closez').classList.add('show');
 const lc=document.getElementById('lidar'),c3=document.getElementById('cloud3d');
 if(card.contains(lc)){orig.lc=[lc.width,lc.height];
   const s=Math.floor(Math.min(innerHeight*0.85,innerWidth*0.85));lc.width=s;lc.height=s;}
 if(card.contains(c3)){orig.c3=[c3.width,c3.height,sc];
   c3.width=Math.floor(innerWidth*0.82);c3.height=Math.floor(innerHeight*0.82);
   sc=Math.min(c3.width,c3.height)/10;draw3d();}}
function closeZoom(){if(!zoomed)return;
 const lc=document.getElementById('lidar'),c3=document.getElementById('cloud3d');
 if(orig.lc&&zoomed.contains(lc)){lc.width=orig.lc[0];lc.height=orig.lc[1];}
 if(orig.c3&&zoomed.contains(c3)){c3.width=orig.c3[0];c3.height=orig.c3[1];sc=orig.c3[2];draw3d();}
 zoomed.classList.remove('zoom');zoomed=null;
 document.getElementById('closez').classList.remove('show');orig={};}
addEventListener('keydown',e=>{if(e.key==='Escape')closeZoom();});
// poll
async function poll(){try{const s=await(await fetch('/state')).json();
 const f=s.fused||{},t=s.tel||{},d=s.drive||{},g=s.gps||{};
 const stx=document.getElementById('state');
 stx.textContent=f.blocked?'BLOCKED':'CLEAR';stx.className='big '+(f.blocked?'blk':'ok');
 document.getElementById('src').textContent='fusion: '+(f.source||'—');
 document.getElementById('near').textContent=f.nearest!=null?f.nearest+' m':'—';
 document.getElementById('speed').textContent=t.speed!=null?t.speed:'—';
 document.getElementById('head').textContent=t.heading!=null?t.heading:'—';
 document.getElementById('yaw').textContent=t.odo?t.odo.yaw:'—';
 document.getElementById('duty').textContent=t.duty!=null?t.duty:'—';
 document.getElementById('erpm').textContent=t.erpm!=null?t.erpm:'—';
 document.getElementById('vin').textContent=t.v_in!=null?t.v_in:'—';
 document.getElementById('temp').textContent=t.temp!=null?t.temp:'—';
 document.getElementById('dist').textContent=t.odo?t.odo.dist:'—';
 document.getElementById('fault').textContent=t.fault||'—';
 document.getElementById('dduty').textContent=d.duty_cmd!=null?d.duty_cmd:'—';
 document.getElementById('gps').textContent=(g.fix?('fix '+g.fix+' / '+g.sats+' sats'):
   (g.lat?'fix':'no fix'));
 const armed=d.armed,es=d.estop,sd=d.selfdrive;
 const ap=document.getElementById('armpill');
 ap.textContent=es?'E-STOP':(sd?('SELF-DRIVE: '+(d.mode||'')):(armed?'ARMED':'DISARMED'));
 ap.style.background=es?'#b3261e':((sd||armed)?'#123a24':'transparent');
 document.getElementById('armbtn').textContent=armed?'DISARM':'ARM';
 const sb=document.getElementById('sdbtn');
 if(sb){sb.textContent=sd?'■ STOP SELF-DRIVE':'▶ SELF-DRIVE';sb.className=sd?'on':'';}
 const br=s.brain||{};
 const bm=document.getElementById('bmode');
 bm.textContent=br.mode||'—';
 bm.className='big '+(br.mode==='DRIVE'?'ok':(br.mode==='HOLD'?'blk':''));
 document.getElementById('breason').textContent=br.reason||'';
 document.getElementById('brear').textContent=br.rear_near!=null?
   ((br.rear_blocked?'BLK ':'clr ')+br.rear_near+'m'):(br.rear_blocked?'BLK':'clr');
 document.getElementById('bopen').textContent=br.best_bearing!=null?br.best_bearing:'—';
 document.getElementById('bpf').textContent=br.person_front?'YES':'no';
 document.getElementById('bpr').textContent=br.person_rear?'YES':'no';
 drawLidar(s);
 }catch(e){}
 setTimeout(poll,150);}
document.getElementById('front').src='/front.mjpg';
document.getElementById('rear').src='/rear.mjpg';
poll();cloud();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype="application/json"):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _mjpeg(self, holder):
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                jpg = holder[0]
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg + b"\r\n")
                time.sleep(0.07)
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_GET(self):
        p = urlparse(self.path).path
        if p == "/":
            self._send(PAGE.encode(), "text/html; charset=utf-8")
        elif p == "/front.mjpg":
            self._mjpeg(FRONT_JPEG)
        elif p == "/rear.mjpg":
            self._mjpeg(REAR_JPEG)
        elif p == "/state":
            with S_LOCK:
                self._send(json.dumps(STATE).encode())
        elif p == "/cloud":
            self._send(json.dumps(HUB.build_cloud()).encode())
        else:
            self.send_error(404)

    def do_POST(self):
        q = parse_qs(urlparse(self.path).query)
        if "estop" in q:
            with C_LOCK:
                CTRL["estop"] = True; CTRL["armed"] = False; CTRL["throttle"] = 0.0
                CTRL["selfdrive"] = False
        if "selfdrive" in q:
            with C_LOCK:
                CTRL["selfdrive"] = not CTRL["selfdrive"]
                if CTRL["selfdrive"]:
                    CTRL["estop"] = False           # enabling self-drive clears e-stop
                else:
                    CTRL["throttle"] = 0.0
        if "arm" in q:
            with C_LOCK:
                if q["arm"][0] == "toggle":
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
        self._send(b"{}")


def main():
    global HUB
    HUB = Hub().start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), H)
    print(f"\nControl Center:  http://localhost:{HTTP_PORT}  "
          f"(or http://<jetson-ip>:{HTTP_PORT})  — hostname -I")
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
