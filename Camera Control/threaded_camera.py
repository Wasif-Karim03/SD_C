#!/usr/bin/env python3
"""
threaded_camera.py — low-latency, always-newest-frame USB capture.

For a self-driving car the perception loop must (a) never block on camera I/O
and (b) always act on the MOST RECENT frame — a stale buffered frame means the
car reacts to where the road WAS, not where it is. This class runs the camera
in a background thread that continuously grabs frames and keeps only the latest;
the main loop's inference then overlaps with the next frame's capture instead of
waiting for it.

Design choices for a control loop:
  - Latest-frame-wins: intermediate frames are intentionally dropped, minimizing
    glass-to-decision latency (throughput-of-newest > processing-every-frame).
  - A monotonically increasing sequence id lets the consumer block until a
    genuinely new frame arrives (read(wait=True)) rather than re-processing one.
  - Small driver buffer (BUFFERSIZE=1) plus continuous draining keeps latency low.

Reuses the project's camera detection + manual-exposure handling from
live_preview.py so framing/exposure stays consistent across all scripts.
"""

import re
import threading
import time

import cv2

import live_preview as lp


class ThreadedCamera:
    def __init__(self, width=None, height=None, apply_exposure=True):
        self.dev, self.info = lp.detect_usb_camera()
        index = int(re.sub(r"\D", "", self.dev) or 0)
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open {self.dev}. Another program (preview/detect) "
                "may still hold the camera — only one V4L2 client is allowed.")

        # Resolution: explicit, else largest supported up to 1280x720.
        if width and height:
            target = (width, height)
        elif self.info and self.info["resolutions"]:
            target = max((r for r in self.info["resolutions"]
                          if r[0] <= 1280 and r[1] <= 720),
                         default=self.info["resolutions"][-1])
        else:
            target = None
        if target:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, target[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target[1])
        # Keep the driver buffer shallow so we don't accumulate latency.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if apply_exposure and lp.USE_MANUAL_EXPOSURE:
            lp.apply_manual_exposure(self.dev, lp.EXPOSURE)

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self._frame = None
        self._seq = 0
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._running = False
        self._thread = None
        self._read_fail = 0

    def start(self):
        if self._running:
            return self
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        # Wait briefly for the first frame so callers get valid data immediately.
        t0 = time.monotonic()
        while self._seq == 0 and time.monotonic() - t0 < 5.0:
            time.sleep(0.005)
        return self

    def _loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self._read_fail += 1
                if self._read_fail > 100:
                    self._running = False
                continue
            self._read_fail = 0
            with self._cond:
                self._frame = frame
                self._seq += 1
                self._cond.notify_all()

    def read(self, wait=True, last_seq=None, timeout=1.0):
        """Return (frame, seq) of the newest frame.

        wait=True blocks until a frame newer than last_seq is available, so the
        consumer never re-processes the same frame and naturally paces to the
        camera while inference overlaps capture. Returns (None, last_seq) on
        timeout.
        """
        with self._cond:
            if wait:
                target = (last_seq if last_seq is not None else self._seq)
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


if __name__ == "__main__":
    # Quick self-test: measure raw capture FPS the threaded way (no inference).
    cam = ThreadedCamera().start()
    print(f"Opened {cam.dev} at {cam.width}x{cam.height}")
    seq = 0
    t0 = time.monotonic()
    n = 0
    while n < 120:
        frame, seq = cam.read(wait=True, last_seq=seq)
        if frame is not None:
            n += 1
    dt = time.monotonic() - t0
    print(f"threaded capture: {n/dt:.1f} FPS over {n} fresh frames")
    cam.release()
