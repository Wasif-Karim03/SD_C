#!/usr/bin/env python3
"""
live_cameras.py — watch BOTH cameras live in a web browser (no display needed).

Serves an MJPEG stream of the front + rear cameras side by side over plain
http.server (no framework, same spirit as GPS/gps_web.py). Open it from the
Jetson itself, or from your phone/laptop on the same Wi-Fi.

Cameras are pinned by STABLE physical-USB-port path (/dev/v4l/by-path), not by
/dev/videoN — the numbers can swap on replug, the port can't.

Run on the Jetson:
    cd "/home/wasif/Documents/Self Driving Car/robocar/apps"
    python3 live_cameras.py
    # then open  http://<jetson-ip>:8090   (find the IP with:  hostname -I)
    # on the Jetson itself:  http://localhost:8090
    # Ctrl-C to stop.

NOTE: this opens both cameras, and only ONE program may hold a camera at a time.
Stop anything else using them (autonav, the probe) before running, and stop this
before running autonomy.
"""
import os
import re
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2

HTTP_PORT = 8090
WIDTH, HEIGHT = 640, 480
STREAM_FPS = 15
JPEG_QUALITY = 80

# name, stable by-path symlink (physical USB port), rotation.
# The rear camera is mounted upside down -> rotate 180 so its view is upright.
# (This same rotation will be recorded in config/hardware.py for the real code.)
CAMERAS = {
    "front": ("Front  (USB port 2.1)",
              "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0",
              None),
    "rear":  ("Rear  (USB port 2.3)",
              "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0",
              cv2.ROTATE_180),
}


def resolve_index(by_path, fallback_dev):
    """by-path symlink -> /dev/videoN -> integer index for cv2."""
    dev = fallback_dev
    if os.path.exists(by_path):
        dev = os.path.realpath(by_path)
    m = re.search(r"(\d+)$", dev)
    return int(m.group(1)) if m else 0, dev


class CameraStream:
    """Background grabber keeping only the newest frame (low latency)."""

    def __init__(self, key, label, by_path, fallback_dev, rotate=None):
        self.key = key
        self.label = label
        self.rotate = rotate       # None, or a cv2.ROTATE_* code
        self.index, self.dev = resolve_index(by_path, fallback_dev)
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        self.ok = self.cap.isOpened()
        if self.ok:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._frame = None
        self._lock = threading.Lock()
        self._run = False

    def start(self):
        if not self.ok:
            print(f"  [{self.key}] FAILED to open {self.dev}")
            return self
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"  [{self.key}] {self.label} -> {self.dev} (index {self.index})")
        return self

    def _loop(self):
        while self._run:
            ok, f = self.cap.read()
            if ok and f is not None:
                if self.rotate is not None:
                    f = cv2.rotate(f, self.rotate)
                with self._lock:
                    self._frame = f
            else:
                time.sleep(0.01)

    def jpeg(self):
        with self._lock:
            f = None if self._frame is None else self._frame.copy()
        if f is None:
            return None
        ok, buf = cv2.imencode(".jpg", f,
                               [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        return buf.tobytes() if ok else None

    def stop(self):
        self._run = False
        if self.ok:
            self.cap.release()


STREAMS = {}

PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>RoboCar — Live Cameras</title>
<style>
 body{{margin:0;background:#0d1117;color:#e6edf3;
   font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;text-align:center}}
 h1{{font-size:1.1rem;font-weight:600;padding:14px 0 4px;margin:0}}
 .hint{{color:#9fb0be;font-size:.85rem;margin:0 0 12px}}
 .wrap{{display:flex;flex-wrap:wrap;gap:16px;justify-content:center;padding:8px 12px 24px}}
 .cam{{background:#141c24;border:1px solid #243140;border-radius:12px;padding:10px}}
 .cam h2{{font-size:.9rem;font-weight:600;margin:0 0 8px;color:#e39b4a}}
 img{{width:min(90vw,640px);height:auto;border-radius:8px;background:#000;display:block}}
</style></head><body>
<h1>RoboCar — Live Cameras</h1>
<p class=hint>Tip: cover the FRONT lens with your hand — whichever feed goes dark is the front camera.</p>
<div class=wrap>{cards}</div>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # quiet

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            cards = ""
            for key, s in STREAMS.items():
                state = "" if s.ok else " — NOT AVAILABLE"
                cards += (f'<div class=cam><h2>{s.label}{state}</h2>'
                          f'<img src="/stream?cam={key}" alt="{key}"></div>')
            body = PAGE.format(cards=cards).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if u.path == "/stream":
            key = parse_qs(u.query).get("cam", [""])[0]
            s = STREAMS.get(key)
            if s is None or not s.ok:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                period = 1.0 / STREAM_FPS
                while True:
                    jpg = s.jpeg()
                    if jpg:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError):
                return  # browser closed the tab
            return

        self.send_error(404)


def main():
    print("Opening cameras ...")
    for key, (label, by_path, rotate) in CAMERAS.items():
        STREAMS[key] = CameraStream(key, label, by_path,
                                    f"/dev/video{0 if key=='front' else 2}",
                                    rotate=rotate).start()
    if not any(s.ok for s in STREAMS.values()):
        raise SystemExit("No cameras opened — are they plugged in / in use by "
                         "another program?")
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"\nLive at:  http://localhost:{HTTP_PORT}   "
          f"(or http://<jetson-ip>:{HTTP_PORT} from your phone/PC)")
    print("Find the Jetson IP with:  hostname -I")
    print("Ctrl-C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping ...")
    finally:
        srv.shutdown()
        for s in STREAMS.values():
            s.stop()
        print("cameras released.")


if __name__ == "__main__":
    main()
