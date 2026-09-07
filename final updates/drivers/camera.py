#!/usr/bin/env python3
"""
drivers/camera.py — threaded latest-frame USB camera (front or rear).

Two identical icSpring cameras, pinned by physical USB port path from config
(front = port 2.1, rear = port 2.3). The rear camera is mounted upside down, so
its frames are auto-rotated 180deg (from config.CAM_REAR_ROTATE) — every consumer
gets an upright image without thinking about it.

Latest-frame-wins: a background thread grabs continuously and keeps only the
newest frame, so the perception loop never acts on a stale image and never blocks
on camera I/O.

  cam = Camera("front").start()
  frame, seq = cam.read(wait=True, last_seq=seq)
  cam.release()
"""
import os
import re
import sys
import threading
import time

import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # noqa: E402

_ROT = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def _resolve_index(by_path, fallback_dev):
    """Physical-port by-path symlink -> /dev/videoN -> integer index for cv2."""
    dev = os.path.realpath(by_path) if os.path.exists(by_path) else fallback_dev
    m = re.search(r"(\d+)$", dev)
    return (int(m.group(1)) if m else 0), dev


class Camera:
    def __init__(self, which="front", width=None, height=None):
        which = which.lower()
        if which == "front":
            by_path, fb, self.rotate = (config.CAM_FRONT_BYPATH,
                                        config.CAM_FRONT_FALLBACK,
                                        config.CAM_FRONT_ROTATE)
        elif which == "rear":
            by_path, fb, self.rotate = (config.CAM_REAR_BYPATH,
                                        config.CAM_REAR_FALLBACK,
                                        config.CAM_REAR_ROTATE)
        else:
            raise ValueError("which must be 'front' or 'rear'")
        self.which = which
        self.index, self.dev = _resolve_index(by_path, fb)
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"could not open {which} camera ({self.dev}). In use by another "
                "program? (only one V4L2 client per camera)")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width or config.CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height or config.CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._rot = _ROT.get(self.rotate)
        self._frame = None
        self._seq = 0
        self._cond = threading.Condition()
        self._running = False
        self._thread = None
        self._fail = 0

    def start(self):
        if self._running:
            return self
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        t0 = time.monotonic()
        while self._seq == 0 and time.monotonic() - t0 < 5.0:
            time.sleep(0.005)
        return self

    def _loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self._fail += 1
                if self._fail > 100:
                    self._running = False
                continue
            self._fail = 0
            if self._rot is not None:
                frame = cv2.rotate(frame, self._rot)
            with self._cond:
                self._frame = frame
                self._seq += 1
                self._cond.notify_all()

    def read(self, wait=True, last_seq=None, timeout=1.0):
        """Return (frame, seq) of the newest frame (already upright)."""
        with self._cond:
            if wait:
                target = last_seq if last_seq is not None else self._seq
                if not self._cond.wait_for(lambda: self._seq > target, timeout):
                    return None, (last_seq if last_seq is not None else self._seq)
            if self._frame is None:
                return None, self._seq
            return self._frame, self._seq

    def release(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.cap.release()

    def __enter__(self):
        return self.start()

    def __exit__(self, *a):
        self.release()


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "front"
    cam = Camera(which).start()
    print(f"opened {which}: {cam.dev} @ {cam.width}x{cam.height} rotate={cam.rotate}")
    seq, n, t0 = 0, 0, time.monotonic()
    while n < 60:
        f, seq = cam.read(wait=True, last_seq=seq)
        if f is not None:
            n += 1
    print(f"captured {n} frames @ {n/(time.monotonic()-t0):.1f} fps")
    cam.release()
