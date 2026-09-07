#!/usr/bin/env python3
"""
apps/dashboard.py — LIVE visual dashboard in your browser (no motors).

One page that shows, in real time:
  * the FRONT CAMERA with the perception overlay (free-space bars, heading arrow,
    depth heatmap inset),
  * a top-down LIDAR MAP of the room (green points, forward = up, front cone, and
    obstacles inside the stop zone in red) — the SLAM-style live view,
  * a STATUS panel: FUSED clear/blocked, steer, nearest-obstacle distance, and what
    each sensor (camera vs LiDAR) is saying.

Nothing drives — this is purely to SEE what the car perceives.

Run on the Jetson (front camera + LiDAR free; stop other camera/lidar users):
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 dashboard.py
Then open in a browser:
    http://<jetson-ip>:8091      (phone/PC on same Wi-Fi;  hostname -I for the IP)
    http://localhost:8091        (on the Jetson)
Ctrl-C to stop.
"""
import os
import sys
import json
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import config                                    # noqa: E402
from drivers.camera import Camera                # noqa: E402
from drivers.lidar import ThreadedLidar          # noqa: E402
from perception.fusion import FusedNavigator     # noqa: E402

HTTP_PORT = 8091


class Perception:
    """Background thread: camera + lidar -> fused decision + annotated JPEG + state."""

    def __init__(self):
        self.jpeg = None
        self.state = {}
        self.lock = threading.Lock()
        self.running = False
        self.cam = None
        self.lid = None
        self.nav = None

    def start(self):
        print("Opening front camera ...")
        self.cam = Camera("front").start()
        print("Starting LiDAR ...")
        try:
            self.lid = ThreadedLidar().start()
        except Exception as exc:  # noqa: BLE001
            print(f"  (LiDAR unavailable: {exc} — camera-only)")
            self.lid = None
        print("Loading depth model (first load is slow) ...")
        self.nav = FusedNavigator(device=0)
        self.nav.estimate_camera(np.zeros((self.cam.height, self.cam.width, 3),
                                           np.uint8))
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print("Perception running.")
        return self

    def _loop(self):
        seq = 0
        while self.running:
            frame, seq = self.cam.read(wait=True, last_seq=seq)
            if frame is None:
                continue
            frame = frame.copy()
            scan = None
            if self.lid is not None:
                s, age = self.lid.latest()
                scan = s if (s and age < 0.5) else None
            fused = self.nav.plan(frame, scan)
            cam, lid = fused["camera"], fused["lidar"]
            annotated = self.nav.cam_nav.draw(frame, fused["cam_depth"], cam)
            ok, buf = cv2.imencode(".jpg", annotated,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            pts = []
            if scan:
                for _q, a, dmm in scan:
                    d = dmm / 1000.0
                    if d > 0:
                        pts.append([round(a, 1), round(d, 2)])
            st = {
                "blocked": bool(fused["blocked"]),
                "steer": round(float(fused["steer"]), 2),
                "nearest": (round(fused["nearest_ahead_m"], 2)
                            if fused["nearest_ahead_m"] is not None else None),
                "source": fused["source"],
                "cam_blocked": bool(cam["blocked"]),
                "cam_steer": round(float(cam["steer"]), 2),
                "cam_reach": round(float(cam["center_reach"]), 2),
                "lidar_blocked": (bool(lid["blocked"]) if lid else None),
                "lidar_steer": (round(float(lid["steer"]), 2) if lid else None),
                "forward_deg": self.nav.lidar_nav.forward_deg,
                "front_arc": self.nav.lidar_nav.front_arc,
                "stop_m": self.nav.lidar_nav.stop_m,
                "points": pts,
            }
            with self.lock:
                if ok:
                    self.jpeg = buf.tobytes()
                self.state = st

    def snapshot(self):
        with self.lock:
            return self.jpeg, json.dumps(self.state)

    def stop(self):
        self.running = False
        time.sleep(0.2)
        if self.lid:
            self.lid.stop()
        if self.cam:
            self.cam.release()


PERC = None

PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>RoboCar — Live Perception</title>
<style>
 body{margin:0;background:#0d1117;color:#e6edf3;font-family:system-ui,Segoe UI,Roboto,sans-serif}
 h1{font-size:1rem;font-weight:600;margin:0;padding:12px 16px;border-bottom:1px solid #243140}
 .wrap{display:flex;flex-wrap:wrap;gap:16px;padding:16px;align-items:flex-start}
 .card{background:#141c24;border:1px solid #243140;border-radius:12px;padding:12px}
 .card h2{font-size:.8rem;font-weight:600;margin:0 0 8px;color:#e39b4a;text-transform:uppercase;letter-spacing:.05em}
 img,canvas{display:block;border-radius:8px;background:#000}
 #status{display:flex;gap:24px;align-items:center;flex-wrap:wrap}
 .big{font-size:1.6rem;font-weight:700}
 .ok{color:#4fbf8f}.blk{color:#e8735a}
 .row{display:flex;gap:18px;flex-wrap:wrap;font-size:.9rem;color:#9fb0be}
 .row b{color:#e6edf3}
 .bar{width:180px;height:12px;background:#22303c;border-radius:6px;position:relative;overflow:hidden}
 .bar i{position:absolute;top:0;bottom:0;background:#5ab0da;width:2px;left:50%}
</style></head><body>
<h1>RoboCar — Live Perception &amp; Fusion <span style=color:#6b7c8a>(no motors)</span></h1>
<div class=wrap>
 <div class=card><h2>Front camera + free-space</h2><img id=cam width=640 height=480></div>
 <div class=card><h2>LiDAR map (top-down, forward = up)</h2>
   <canvas id=lidar width=440 height=440></canvas></div>
 <div class=card style="min-width:280px">
   <h2>Fused decision</h2>
   <div id=status>
     <div><div id=state class=big>—</div><div style=color:#6b7c8a;font-size:.8rem id=src></div></div>
   </div>
   <div style=height:14px></div>
   <div class=row><div>nearest ahead<br><b id=near>—</b></div>
     <div>steer<br><div class=bar><i id=steer></i></div></div></div>
   <div style=height:14px></div>
   <div class=row>
     <div>camera<br><b id=cam_s>—</b></div>
     <div>lidar<br><b id=lid_s>—</b></div>
   </div>
 </div>
</div>
<script>
const cam=document.getElementById('cam');
cam.src='/camera.mjpg';
const cv=document.getElementById('lidar'),cx=cv.getContext('2d');
const CX=cv.width/2,CY=cv.height/2,MAXR=5.0,SCALE=(cv.width/2-14)/MAXR;
function draw(s){
 cx.fillStyle='#0b0f14';cx.fillRect(0,0,cv.width,cv.height);
 // range rings
 cx.strokeStyle='#22303c';cx.fillStyle='#40506050';
 for(let r=1;r<=MAXR;r++){cx.beginPath();cx.arc(CX,CY,r*SCALE,0,7);cx.stroke();}
 // front cone
 const fa=(s.front_arc||60)*Math.PI/180;
 cx.fillStyle='rgba(227,155,74,.10)';cx.beginPath();cx.moveTo(CX,CY);
 cx.arc(CX,CY,MAXR*SCALE,-Math.PI/2-fa/2,-Math.PI/2+fa/2);cx.closePath();cx.fill();
 // points (forward=up). rel bearing = a-forward
 const fwd=s.forward_deg||0, stop=s.stop_m||0.5;
 (s.points||[]).forEach(p=>{
   let rel=(p[0]-fwd)*Math.PI/180, d=p[1];
   let x=CX+d*SCALE*Math.sin(rel), y=CY-d*SCALE*Math.cos(rel);
   let inFront=Math.abs(((p[0]-fwd+540)%360)-180)<=(s.front_arc||60)/2;
   cx.fillStyle=(inFront&&d<stop)?'#e8735a':'#4fbf8f';
   cx.fillRect(x-1.5,y-1.5,3,3);
 });
 // car
 cx.fillStyle='#5ab0da';cx.beginPath();
 cx.moveTo(CX,CY-9);cx.lineTo(CX-6,CY+7);cx.lineTo(CX+6,CY+7);cx.closePath();cx.fill();
}
async function poll(){
 try{const s=await (await fetch('/state')).json();
   const st=document.getElementById('state');
   st.textContent=s.blocked?'BLOCKED':'CLEAR';
   st.className='big '+(s.blocked?'blk':'ok');
   document.getElementById('src').textContent='source: '+(s.source||'—');
   document.getElementById('near').textContent=(s.nearest!=null?s.nearest+' m':'—');
   document.getElementById('steer').style.left=(50+ (s.steer||0)*48)+'%';
   document.getElementById('cam_s').textContent=(s.cam_blocked?'BLOCKED':'clear')+
     '  st'+(s.cam_steer>=0?'+':'')+s.cam_steer+'  '+s.cam_reach+'m';
   document.getElementById('lid_s').textContent=(s.lidar_blocked==null?'—':
     (s.lidar_blocked?'BLOCKED':'clear')+'  st'+(s.lidar_steer>=0?'+':'')+s.lidar_steer);
   draw(s);
 }catch(e){}
 setTimeout(poll,100);
}
poll();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/state":
            _, js = PERC.snapshot()
            body = js.encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/camera.mjpg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    jpg, _ = PERC.snapshot()
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.06)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_error(404)


def main():
    global PERC
    PERC = Perception().start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"\nDashboard live:  http://localhost:{HTTP_PORT}   "
          f"(or http://<jetson-ip>:{HTTP_PORT})")
    print("Find the Jetson IP with:  hostname -I")
    print("Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping ...")
    finally:
        srv.shutdown()
        PERC.stop()
        print("stopped.")


if __name__ == "__main__":
    sys.exit(main())
