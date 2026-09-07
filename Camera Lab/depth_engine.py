#!/usr/bin/env python3
"""
depth_engine.py — Depth-Anything V2 (small) metric depth on the Jetson.

Step 1 of the on-car 3D mapping pipeline. Unlike MiDaS (which gives *relative*
inverse depth), we use the Depth-Anything V2 **metric** variant so each pixel is
an actual distance in METERS. Metric depth is what lets us later project frames
into a single, correctly-scaled 3D map instead of a scale-drifting one.

Two model choices (indoor default — the car runs indoors):
  - depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf  (0-20 m range)
  - depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf (0-80 m range)

The small ViT-S backbone is the only one worth running on an 8 GB Orin Nano.
This module just does image -> metric depth (HxW float32, meters) and can
self-benchmark FPS so we know the framerate budget for the rest of the pipeline.
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


def _benchmark(n=60, res=(640, 480), model_id=INDOOR, half=True):
    print(f"Loading {model_id} (half={half}) ...")
    eng = DepthEngine(model_id=model_id, half=half)
    dummy = np.random.randint(0, 255, (res[1], res[0], 3), dtype=np.uint8)

    # Warmup (first calls include CUDA/cuDNN autotune — not representative).
    for _ in range(5):
        eng.infer(dummy)
    torch.cuda.synchronize()

    t0 = time.time()
    for _ in range(n):
        d = eng.infer(dummy)
    torch.cuda.synchronize()
    dt = time.time() - t0

    fps = n / dt
    used = torch.cuda.max_memory_allocated() / 1e9
    print(f"depth range: {d.min():.2f}..{d.max():.2f} m  (shape {d.shape})")
    print(f"FPS: {fps:.1f}  |  {1000*dt/n:.1f} ms/frame  |  peak VRAM {used:.2f} GB")
    return fps


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Depth-Anything V2 metric depth / benchmark")
    ap.add_argument("--bench", action="store_true", help="run FPS benchmark")
    ap.add_argument("--n", type=int, default=60, help="benchmark frames")
    ap.add_argument("--outdoor", action="store_true", help="use outdoor metric model")
    ap.add_argument("--fp32", action="store_true", help="disable half precision")
    args = ap.parse_args()

    mid = OUTDOOR if args.outdoor else INDOOR
    if args.bench:
        _benchmark(n=args.n, model_id=mid, half=not args.fp32)
    else:
        print("Use --bench to benchmark. Import DepthEngine to use in the pipeline.")
