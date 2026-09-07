#!/usr/bin/env python3
"""
perception/detect.py — object detection (YOLO11n, TensorRT) for front OR rear camera.

Semantics on top of the geometry: what is that thing — a person, a chair, a box?
Runs the TensorRT engine built in the old work (fast, ~5-7 ms/frame on the Orin),
falling back to the .pt weights if the engine won't load.

  det = Detector()
  dets = det.detect(frame_bgr)      # -> [{name, cls, conf, box:[x1,y1,z2,y2], vru}]
  det.draw(frame, dets)             # boxes + labels (people/VRUs in red)

Give a detection a DISTANCE later by sampling depth or LiDAR inside its box — a box
without range can't drive a speed decision.
"""
import os
import sys

import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import config   # noqa: E402


class Detector:
    def __init__(self, engine=None, pt=None, conf=None, imgsz=None):
        from ultralytics import YOLO
        engine = engine or config.YOLO_ENGINE
        pt = pt or config.YOLO_PT
        self.conf = conf if conf is not None else config.YOLO_CONF
        self.imgsz = imgsz or config.YOLO_IMGSZ
        self.kind = None
        self.model = None
        if os.path.exists(engine):
            try:
                self.model = YOLO(engine, task="detect")
                self.kind = "engine"
            except Exception as exc:  # noqa: BLE001
                print(f"  (YOLO engine load failed: {exc} — trying .pt)")
        if self.model is None:
            self.model = YOLO(pt)
            self.kind = "pt"
        self.names = self.model.names

    def detect(self, frame_bgr):
        r = self.model.predict(frame_bgr, imgsz=self.imgsz, conf=self.conf,
                               verbose=False)[0]
        dets = []
        for b in r.boxes:
            cls = int(b.cls[0])
            dets.append({
                "cls": cls,
                "name": self.names.get(cls, str(cls)),
                "conf": float(b.conf[0]),
                "box": [float(x) for x in b.xyxy[0].tolist()],
                "vru": cls in config.VRU_CLASSES,
            })
        return dets

    def draw(self, frame, dets):
        for d in dets:
            x1, y1, x2, y2 = [int(v) for v in d["box"]]
            col = (0, 0, 255) if d["vru"] else (0, 200, 0)   # VRUs red
            cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
            label = f"{d['name']} {d['conf']*100:.0f}%"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), col, -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return frame
