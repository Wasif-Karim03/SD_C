#!/usr/bin/env python3
"""
apps/mapper_web.py — Create-Map mode: manually drive from the browser while 2D
LiDAR SLAM builds a live indoor map, then save it.

LEAN on purpose: LiDAR + VESC + steering only (NO cameras/depth/YOLO), so it runs
light and won't run the Jetson out of memory. Owns the VESC + steering (run it
instead of control_center / drive.py).

  cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
  python3 mapper_web.py
  open  http://<jetson-ip>:8080

Drive SLOWLY (hold-to-drive deadman + steering slider). Watch the map build.
Click SAVE MAP when the room's covered -> saves maps/room.npy + .png.
E-STOP latches the motor off. Ctrl-C to quit.
"""
import os
import sys
import json
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                 # noqa: E402
from drivers.lidar import RPLidarC1           # noqa: E402
from perception.slam import LidarSLAM         # noqa: E402

HTTP_PORT = 8080
MAXD = config.MAX_DUTY
RAMP = 0.01
DEADMAN_S = 0.4
MAP_DIR = os.path.join(ROOT, "maps")

MAP_PNG = [None]
STATE = {}
S_LOCK = threading.Lock()
CTRL = {"armed": False, "estop": False, "throttle": 0.0, "last_cmd": 0.0}
C_LOCK = threading.Lock()


class Hub:
    def __init__(self):
        self.slam = LidarSLAM()
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
        else:
            print("  vesc n/a (motor battery off?) — you can still map by pushing "
                  "the car slowly by hand.")
        try:
            from drivers.steering import ServoController
            self.steer = ServoController()
            self.steer.center(read_reply=False)
        except Exception as e:  # noqa: BLE001
            print("  steering n/a:", e)
        self.running = True
        threading.Thread(target=self._slam_loop, daemon=True).start()
        threading.Thread(target=self._actuator_loop, daemon=True).start()
        print("mapping. drive slowly.")
        return self

    def _slam_loop(self):
        try:
            n = 0
            for scan in self.lidar.iter_scans(min_points=120):
                if not self.running:
                    break
                self.slam.add_scan(scan)
                n += 1
                if n % 2 == 0:
                    png = self.slam.render_png(out_size=500)
                    if png:
                        MAP_PNG[0] = png
                    with S_LOCK:
                        STATE["pose"] = [round(v, 2) for v in self.slam.pose]
                        STATE["frames"] = self.slam.frames
        except Exception as e:  # noqa: BLE001
            print("slam loop ended:", e)

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
        if self.steer:
            try:
                self.steer.steer(v, read_reply=False)
            except Exception:
                pass

    def save(self):
        path = os.path.join(MAP_DIR, "room")
        self.slam.save(path)
        return path

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
<title>RoboCar — Create Map</title>
<style>
 body{margin:0;background:#0a0e13;color:#e6edf3;font-family:system-ui,sans-serif}
 header{display:flex;align-items:center;justify-content:space-between;padding:10px 16px;
   border-bottom:1px solid #243140}
 header b{font-size:1.05rem}
 button{background:#1a2530;color:#e6edf3;border:1px solid #32424f;border-radius:8px;
   padding:10px 14px;font-size:.95rem;cursor:pointer;margin:3px}
 button:hover{border-color:#5ab0da}
 #estop{background:#b3261e;border:none;font-weight:800;padding:12px 18px}
 #save{background:#123a24;border-color:#2f7d57;font-weight:700}
 .wrap{display:flex;flex-wrap:wrap;gap:16px;padding:16px}
 .card{background:#141c24;border:1px solid #243140;border-radius:12px;padding:12px}
 img{display:block;border-radius:8px;background:#000;width:min(90vw,560px)}
 .go{background:#123a24;border-color:#2f7d57}
 input[type=range]{width:100%}
 .pill{padding:2px 8px;border-radius:999px;font-size:.75rem;border:1px solid #243140}
</style></head><body>
<header><b>🗺️ RoboCar — Create Map (LiDAR SLAM)</b>
 <div><span id=pill class=pill>DISARMED</span>
   <button id=save onclick=save()>💾 SAVE MAP</button>
   <button id=estop onclick=estop()>■ E-STOP</button></div></header>
<div class=wrap>
 <div class=card><h3 style=margin:.2em>Live map</h3>
   <img id=map><div id=info style="color:#9fb0be;font-size:.8rem;margin-top:6px"></div></div>
 <div class=card><h3 style=margin:.2em>Drive (slowly!)</h3>
   <button onclick=arm()>ARM / DISARM</button><br>
   <button class=go onmousedown="hold(1)" onmouseup=rel() onmouseleave=rel()
     ontouchstart="hold(1)" ontouchend=rel()>▲ HOLD FWD</button>
   <button class=go onmousedown="hold(-1)" onmouseup=rel() onmouseleave=rel()
     ontouchstart="hold(-1)" ontouchend=rel()>▼ HOLD REV</button>
   <div style=margin:10px 0>level <b id=lvl>6</b>%
     <button onclick=lvl(-1)>−</button><button onclick=lvl(1)>+</button></div>
   <div>steering<br><input id=st type=range min=-1 max=1 step=0.02 value=0
     oninput=steer(this.value)><button onclick="document.getElementById('st').value=0;steer(0)">center</button></div>
   <p style="color:#9fb0be;font-size:.75rem">Drive SLOWLY so scans overlap (that's
    what keeps the map clean). Cover the whole room, then SAVE MAP.</p></div>
</div>
<script>
let L=6,held=0,ka=null;
async function cmd(q){try{await fetch('/cmd?'+q,{method:'POST'})}catch(e){}}
function arm(){cmd('arm=toggle')} function estop(){cmd('estop=1')}
function lvl(d){L=Math.max(1,Math.min(20,L+d));document.getElementById('lvl').textContent=L}
function steer(v){cmd('steer='+v)}
function hold(d){held=d;s();ka=setInterval(s,150)}
function rel(){held=0;clearInterval(ka);cmd('throttle=0')}
function s(){cmd('throttle='+(held*L/100))}
async function save(){await cmd('save=1');document.getElementById('info').textContent='map saved to maps/room.png';}
function refresh(){document.getElementById('map').src='/map.png?'+Date.now();}
async function poll(){try{const s=await(await fetch('/state')).json();
  const d=s.drive||{};
  document.getElementById('pill').textContent=d.estop?'E-STOP':(d.armed?'ARMED':'DISARMED');
  document.getElementById('info').textContent=
    'frames: '+(s.frames||0)+'  pose: '+(s.pose?s.pose.join(', '):'-')+'  duty '+(d.duty||0)+'%';
 }catch(e){}
 setTimeout(poll,300);}
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
            png = MAP_PNG[0]
            if png:
                self._send(png, "image/png")
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
            with C_LOCK:
                CTRL["estop"] = True; CTRL["armed"] = False; CTRL["throttle"] = 0.0
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
        if "save" in q:
            try:
                print("saved ->", HUB.save())
            except Exception as e:  # noqa: BLE001
                print("save failed:", e)
        self._send(b"{}", "application/json")


def main():
    global HUB
    HUB = Hub().start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), H)
    print(f"\nCreate-Map:  http://localhost:{HTTP_PORT}  (or http://<jetson-ip>:{HTTP_PORT})")
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
