#!/usr/bin/env python3
"""mock.py — stands in for the Jetson so the front end can be rendered and
checked without the car. Serves a plausible NAVIGATE run: a corridor with a
box in it, a plan around the box, and a controller that is lagging the plan."""
import json, math, os, time, io
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
T0 = time.time()

def scan():
    """A 4 m corridor, a doorway on the right, and a 30 cm box at 1.24 m."""
    pts = []
    for i in range(500):
        a = math.radians(i * 360.0 / 500.0)
        fx, fy = math.cos(a), math.sin(a)
        best = None
        for (nx, ny, d) in ((0, 1, 1.15), (0, -1, 1.05), (1, 0, 4.6), (-1, 0, 2.2)):
            den = fx * nx + fy * ny
            if den > 1e-6:
                t = d / den
                if best is None or t < best: best = t
        if best is None or best > 6.0: continue
        # doorway: drop the wall between 1.6 and 2.4 m forward on the right
        if abs(fy) > 0.6 and 1.6 < best * fx < 2.4 and fy < 0: continue
        # the box
        bx, by, bw, bd = 1.34, 0.16, 0.30, 0.24
        if abs(fx) > 1e-6:
            for (sx, sy) in ((bx - bd/2, None), (None, by - bw/2), (None, by + bw/2)):
                pass
        pts.append([round(best * fx, 3), round(best * fy, 3)])
    # box face, sampled
    for k in range(14):
        pts.append([round(1.22 + 0.002 * k, 3), round(0.16 - 0.15 + k * 0.021, 3)])
    pts.sort(key=lambda p: math.atan2(p[1], p[0]))
    return pts

def state():
    t = time.time() - T0
    steer = 0.28 + 0.06 * math.sin(t * 0.6)
    plan = [[0.0, 0.0]]
    for i in range(1, 30):
        f = i * 0.13
        plan.append([round(f, 3), round(-0.42 * math.exp(-((f - 1.5) ** 2) / 0.5), 3)])
    trail = [[-0.9 + i * 0.09, 0.03 * math.sin(i * 0.4)] for i in range(10)]
    return {
        "mode": "navigate", "env": "indoor", "rear_on": True,
        "scan_age": round(0.05 + 0.02 * abs(math.sin(t)), 2),
        "near": round(1.24 + 0.05 * math.sin(t * 0.9), 2),
        "loop_ms": 19.9, "heading": 41,
        "tele": {"v_in": round(15.61 - t * 0.001, 2), "temp_mos": 54.2, "temp_motor": 31.0,
                 "motor_current": 6.4, "erpm": 2100, "tach": 18422,
                 "fault": "FAULT_CODE_NONE", "speed": round(0.31 + 0.03 * math.sin(t), 2)},
        "drive": {"armed": True, "estop": False, "duty": 7.0},
        "ctrl": {"armed": True, "estop": False, "throttle": 0.07, "steer": round(steer, 3)},
        "follow": {"on": True, "note": "driving 3.42m to goal",
                   "heading_err": round(4.8 + math.sin(t) * 1.2, 2),
                   "cross_track": round(0.11 + 0.02 * math.sin(t * 1.3), 3),
                   "steer_pursuit": 0.42, "steer_veto": 0.14, "icp_fail": 0},
        "health": {"loc": True, "lidar": True, "vesc": True, "steer": True,
                   "cam": True, "detector": True},
        "rec": {"on": True, "rows": 4982, "dir": "logs/2026-09-09T14-22-07", "note": "corner regression, run 3 of 20"},
        "front_dets": [[0.14, 0.22, 0.44, 0.79, "person", 0.78],
                       [0.58, 0.44, 0.86, 0.72, "chair", 0.63]],
        "rear_dets": [[0.30, 0.35, 0.62, 0.80, "person", 0.55]],
        "path_local": plan, "trail_local": trail,
        "goal_local": [3.30, -0.88], "goal_dist": 3.42,
        "lookahead_local": [0.62, -0.11],
        "gps": {"fix": True, "lat": 41.516474, "lon": -81.611847, "alt": 199.4,
                "sats": 9, "hdop": 1.1, "mode": "GPS", "course": 41.0, "speed": 0.31,
                "present": True, "errors": 0,
                "satlist": [{"prn": "GP" + str(p), "cn": c, "used": u}
                            for p, c, u in [(5,44,1),(13,41,1),(15,39,1),(20,37,1),(21,34,1),
                                            (24,31,1),(28,28,1),(2,26,1),(6,24,1),(30,19,0),(11,16,0)]]},
        "waypoints_enu": [[6, 9], [14, 13], [21, 6]],
        "session": {"sha": "2029986", "dirty": False,
                    "started": "2026-09-09T14:22:07Z", "epoch_ms": int(T0 * 1000)},
        "config": {"wheelbase": 0.25, "maxSteer": 0.45, "lookahead": 0.55,
                   "goalTol": 0.25, "steerGain": 1.8, "react": 1.3,
                   "forwardDeg": 354.6, "maxDuty": 0.20, "mpt": 0.003424,
                   "est": ["WHEELBASE_M", "MAX_STEER_ANGLE_RAD"]},
    }

JPG = None
def jpg():
    global JPG
    if JPG is None:
        import base64
        # 1x1 grey jpeg is enough: we are checking layout, not imagery
        JPG = base64.b64decode(
            "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0a"
            "HBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAALCAABAAEBAREA/8QAFAABAAAAAAAA"
            "AAAAAAAAAAAACf/EABQQAQAAAAAAAAAAAAAAAAAAAAD/2gAIAQEAAD8AKp//2Q==")
    return JPG

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _s(self, b, c):
        self.send_response(200); self.send_header("Content-Type", c)
        self.send_header("Content-Length", str(len(b))); self.end_headers()
        self.wfile.write(b)
    def do_GET(self):
        p = urlparse(self.path).path
        if p == "/": p = "/web/index.html"
        if p == "/state": return self._s(json.dumps(state()).encode(), "application/json")
        if p == "/scan":  return self._s(json.dumps({"pts": scan()}).encode(), "application/json")
        if p.endswith(".mjpg") or p == "/map.jpg": return self._s(jpg(), "image/jpeg")
        if p.startswith("/web/"):
            f = os.path.normpath(os.path.join(WEB, p[5:]))
            if f.startswith(WEB) and os.path.isfile(f):
                ct = {"html": "text/html", "css": "text/css", "js": "application/javascript"}.get(
                    f.rsplit(".", 1)[-1], "application/octet-stream")
                return self._s(open(f, "rb").read(), ct + "; charset=utf-8")
        self.send_error(404)
    def do_POST(self): self._s(b"{}", "application/json")

if __name__ == "__main__":
    # Say something on start. A server that binds silently is a server you
    # stare at wondering whether it came up.
    print("\nmock cockpit up:  http://localhost:8099")
    print("simulated run - corridor, doorway, 30 cm box at 1.24 m, no hardware")
    print("Ctrl-C to stop.\n", flush=True)
    try:
        ThreadingHTTPServer(("127.0.0.1", 8099), H).serve_forever()
    except KeyboardInterrupt:
        print("stopped.")
