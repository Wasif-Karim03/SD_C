#!/usr/bin/env python3
"""
perception/depth.py — Depth-Anything V2 (small, metric) depth on the Jetson.

Ported from Camera Lab/depth_engine.py. Each pixel is an actual distance in METERS
(not MiDaS relative inverse depth), which is what lets us reason about free space
in real units and later project into a costmap.

Indoor default (0-20 m). ViT-S backbone, fp16 — the only sensible size on an 8 GB
Orin. ~12 fps @640x480 (well within the ~20 fps camera budget). Needs
transformers >= 4.45 for the metric head.
"""
import time

import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

INDOOR = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
OUTDOOR = "depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf"


class DepthEngine:
    def __init__(self, model_id=INDOOR, device=0, half=True):
        self.device = torch.device(f"cuda:{device}")
        self.half = half
        self.processor = AutoImageProcessor.from_pretrained(model_id)
        self.model = AutoModelForDepthEstimation.from_pretrained(model_id)
        self.model = self.model.to(self.device).eval()
        if half:
            self.model = self.model.half()

    @torch.no_grad()
    def infer(self, frame_bgr):
        """BGR frame -> metric depth map (HxW float32, meters), same size as input."""
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt")
        pixel_values = inputs["pixel_values"].to(self.device)
        if self.half:
            pixel_values = pixel_values.half()
        depth = self.model(pixel_values=pixel_values).predicted_depth
        depth = torch.nn.functional.interpolate(
            depth.unsqueeze(1).float(), size=frame_bgr.shape[:2],
            mode="bicubic", align_corners=False).squeeze()
        return depth.cpu().numpy()

    @staticmethod
    def colorize(depth, max_m=None):
        """Metric depth -> BGR heatmap for visual sanity checks (near=warm)."""
        d = depth.copy()
        hi = max_m if max_m else np.percentile(d, 95)
        d = np.clip(d / max(hi, 1e-6), 0, 1)
        d = (d * 255).astype(np.uint8)
        return cv2.applyColorMap(255 - d, cv2.COLORMAP_INFERNO)


if __name__ == "__main__":
    # Quick self-benchmark on random frames.
    print(f"Loading {INDOOR} ...")
    eng = DepthEngine()
    dummy = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    for _ in range(5):
        eng.infer(dummy)
    torch.cuda.synchronize()
    t0 = time.time()
    for _ in range(30):
        d = eng.infer(dummy)
    torch.cuda.synchronize()
    dt = time.time() - t0
    print(f"depth {d.min():.2f}..{d.max():.2f} m  {30/dt:.1f} fps  "
          f"{1000*dt/30:.1f} ms/frame  peak VRAM "
          f"{torch.cuda.max_memory_allocated()/1e9:.2f} GB")
