#!/usr/bin/env python3
"""
apps/map3d.py — live 3D "Tesla-style" view of the world around the car (no motors).

Builds a 3D point cloud of the surroundings from THREE sensors and lets you orbit
it in a browser (2D-canvas renderer — NO WebGL needed, works over NoMachine):
  * FRONT camera metric depth -> colored 3D points ahead
  * REAR  camera metric depth -> colored 3D points behind
  * LiDAR 360deg ring          -> the accurate horizontal wall outline (orange)
The car sits at the origin. Drag to rotate, scroll to zoom. "Recapture" refreshes.

Honest limits: two non-overlapping cameras -> blind wedges on the sides (LiDAR ring
fills those in 2D). Camera intrinsics are ESTIMATED and mono depth scale drifts, so
geometry is approximate until calibrated — but it already looks like a real 3D map.

Run on the Jetson (front+rear cameras + LiDAR free):
    cd "/home/wasif/Documents/Self Driving Car/final updates/apps"
    python3 map3d.py
Then open  http://<jetson-ip>:8092  (or http://localhost:8092). Ctrl-C to stop.
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
import config                                 # noqa: E402
from drivers.camera import Camera             # noqa: E402
from drivers.lidar import ThreadedLidar       # noqa: E402
from perception.depth import DepthEngine      # noqa: E402

HTTP_PORT = 8092
STRIDE = 6
DMIN, DMAX = 0.25, 8.0
FX = FY = 554.0
CAM_H = 0.15
FRONT_X = 0.10
REAR_X = 0.10


class Builder:
    def __init__(self):
        self.front = self.rear = self.lid = self.eng = None
        self.lock = threading.Lock()

    def start(self):
        print("Opening front camera ...")
        self.front = Camera("front").start()
        try:
            print("Opening rear camera ...")
            self.rear = Camera("rear").start()
        except Exception as exc:  # noqa: BLE001
            print(f"  (rear camera unavailable: {exc})")
            self.rear = None
        try:
            print("Starting LiDAR ...")
            self.lid = ThreadedLidar().start()
        except Exception as exc:  # noqa: BLE001
            print(f"  (LiDAR unavailable: {exc})")
            self.lid = None
        print("Loading depth model ...")
        self.eng = DepthEngine(device=0)
        self.eng.infer(np.zeros((self.front.height, self.front.width, 3), np.uint8))
        print("Ready.")
        return self

    def _cam_cloud(self, cam, rear=False):
        if cam is None:
            return []
        frame, _ = cam.read(wait=True)
        if frame is None:
            return []
        depth = self.eng.infer(frame)
        h, w = depth.shape
        cx, cy = w / 2.0, h / 2.0
        pts = []
        for v in range(0, h, STRIDE):
            for u in range(0, w, STRIDE):
                d = float(depth[v, u])
                if d < DMIN or d > DMAX:
                    continue
                X = (u - cx) / FX * d
                Y = (v - cy) / FY * d
                Z = d
                if not rear:
                    cxm, cym, czm = Z + FRONT_X, -X, -Y + CAM_H
                else:
                    cxm, cym, czm = -(Z + REAR_X), X, -Y + CAM_H
                b, g, r = frame[v, u]
                pts.append([round(cxm, 3), round(cym, 3), round(czm, 3),
                            min(255, int(r * 2.2)), min(255, int(g * 2.2)),
                            min(255, int(b * 2.2))])
        return pts

    def _lidar_cloud(self):
        if self.lid is None:
            return []
        scan, age = self.lid.latest()
        if not scan or age > 0.5:
            return []
        fwd = config.LIDAR_FORWARD_DEG
        pts = []
        for _q, a, dmm in scan:
            d = dmm / 1000.0
            if d < config.LIDAR_MIN_M or d > DMAX:
                continue
            rel = np.radians(a - fwd)
            pts.append([round(float(d * np.cos(rel)), 3),
                        round(float(d * np.sin(rel)), 3), 0.08,
                        240, 155, 74])
        return pts

    def capture(self):
        with self.lock:
            cam_pts = self._cam_cloud(self.front, False) + self._cam_cloud(self.rear, True)
            lid = self._lidar_cloud()
            return {"points": cam_pts + lid, "n_cam": len(cam_pts), "n_lidar": len(lid)}

    def stop(self):
        if self.lid:
            self.lid.stop()
        if self.front:
            self.front.release()
        if self.rear:
            self.rear.release()


BUILDER = None

PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>RoboCar — 3D Map</title>
<style>
 body{margin:0;background:#0a0e13;color:#e6edf3;font-family:system-ui,sans-serif;overflow:hidden}
 #hud{position:fixed;top:10px;left:12px;z-index:10;font-size:13px;color:#9fb0be}
 #hud b{color:#e6edf3}
 button{background:#1a2530;color:#e6edf3;border:1px solid #32424f;border-radius:8px;
   padding:8px 14px;font-size:13px;cursor:pointer;margin-top:8px}
 button:hover{border-color:#5ab0da}
 canvas{display:block}
</style></head><body>
<div id=hud><b>RoboCar 3D map</b> — drag to rotate, scroll to zoom (2D renderer)<br>
 <span id=stat>loading…</span><br><button onclick=recapture()>Recapture</button></div>
<canvas id=view></canvas>
<script>
const cv=document.getElementById('view'),ctx=cv.getContext('2d');
let pts=[],az=0.7,el=-0.5,scale=55,drag=false,px,py,dirty=true;
function resize(){cv.width=innerWidth;cv.height=innerHeight;dirty=true;}
addEventListener('resize',resize);resize();
cv.addEventListener('mousedown',e=>{drag=true;px=e.clientX;py=e.clientY});
addEventListener('mouseup',()=>drag=false);
addEventListener('mousemove',e=>{if(!drag)return;
 az-=(e.clientX-px)*0.01;el+=(e.clientY-py)*0.01;
 el=Math.max(-1.5,Math.min(1.5,el));px=e.clientX;py=e.clientY;dirty=true});
cv.addEventListener('wheel',e=>{scale*=(1-Math.sign(e.deltaY)*0.1);
 scale=Math.max(8,Math.min(400,scale));dirty=true;e.preventDefault()},{passive:false});
function proj(x,y,z){
 const ca=Math.cos(az),sa=Math.sin(az);
 const x1=x*ca-y*sa,y1=x*sa+y*ca;
 const ce=Math.cos(el),se=Math.sin(el);
 const z2=y1*se+z*ce;
 return [cv.width/2+x1*scale, cv.height/2-z2*scale];
}
function draw(){
 ctx.fillStyle='#0a0e13';ctx.fillRect(0,0,cv.width,cv.height);
 const O=proj(0,0,0);
 [[1,0,0,'#e8735a'],[0,1,0,'#4fbf8f'],[0,0,1,'#5ab0da']].forEach(a=>{
   const P=proj(a[0],a[1],a[2]);ctx.strokeStyle=a[3];ctx.lineWidth=2;
   ctx.beginPath();ctx.moveTo(O[0],O[1]);ctx.lineTo(P[0],P[1]);ctx.stroke();});
 for(let i=0;i<pts.length;i++){const p=pts[i],s=proj(p[0],p[1],p[2]);
   ctx.fillStyle='rgb('+p[3]+','+p[4]+','+p[5]+')';ctx.fillRect(s[0],s[1],2,2);}
 ctx.fillStyle='#5ab0da';ctx.fillRect(O[0]-4,O[1]-4,8,8);
 dirty=false;
}
function loop(){if(dirty)draw();requestAnimationFrame(loop);}
loop();
async function recapture(){
 document.getElementById('stat').textContent='capturing…';
 try{const d=await (await fetch('/points')).json();
   const all=d.points,step=Math.max(1,Math.floor(all.length/9000));
   pts=all.filter((_,i)=>i%step===0);
   document.getElementById('stat').innerHTML='camera pts: <b>'+d.n_cam+
     '</b> · lidar pts: <b>'+d.n_lidar+'</b> · showing '+pts.length;
   dirty=true;
 }catch(e){document.getElementById('stat').textContent='capture failed: '+e.message;}
}
recapture();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/points":
            data = BUILDER.capture()
            body = json.dumps(data).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)


def main():
    global BUILDER
    BUILDER = Builder().start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"\n3D map live:  http://localhost:{HTTP_PORT}  "
          f"(or http://<jetson-ip>:{HTTP_PORT})   —  hostname -I for the IP")
    print("Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping ...")
    finally:
        srv.shutdown()
        BUILDER.stop()
        print("stopped.")


if __name__ == "__main__":
    sys.exit(main())
