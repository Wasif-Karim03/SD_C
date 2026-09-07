#!/usr/bin/env python3
"""
lane_detect.py — classic-CV lane / drivable-path detection + steering signal.

Gives a self-driving car the "where is the road" signal that object detection
can't: it finds lane/path lines in the lower part of the frame and computes a
STEERING OFFSET = how far the lane center sits from the camera center, in the
range -1.0 (path is to the left) .. +1.0 (path is to the right). 0.0 means the
path is dead ahead. That offset is directly usable as a steering error term.

Pure OpenCV on CPU (no GPU, no extra installs) so it runs alongside the GPU
YOLO detector for free. Tuned for a forward-facing camera looking at a road /
track / taped path; parameters below are the knobs to adjust for your surface.

Standalone:  python3 lane_detect.py [image.jpg]   (defaults to camera live view)
"""

import sys
import numpy as np
import cv2

# --- Tuning knobs ----------------------------------------------------------- #
CANNY_LO, CANNY_HI = 50, 150         # edge sensitivity
ROI_TOP = 0.55                       # ignore everything above this frac of height
ROI_BOTTOM_PAD = 0.0                 # frac of width cropped from bottom corners
HOUGH_THRESH = 30
HOUGH_MIN_LEN = 20
HOUGH_MAX_GAP = 100
MIN_ABS_SLOPE = 0.5                  # reject near-horizontal lines
SMOOTH = 0.4                         # EMA factor for steering offset (0=off)


def _roi_vertices(w, h):
    top = int(h * ROI_TOP)
    pad = int(w * ROI_BOTTOM_PAD)
    return np.array([[(pad, h), (w - pad, h),
                      (int(w * 0.95), top), (int(w * 0.05), top)]], np.int32)


def _avg_line(lines, w, h):
    """Average a set of (x1,y1,x2,y2) into one extrapolated line bottom->ROI top."""
    if not lines:
        return None
    xs, ys = [], []
    for x1, y1, x2, y2 in lines:
        xs += [x1, x2]; ys += [y1, y2]
    # Fit x = m*y + b (y is the stable axis for near-vertical lane lines).
    m, b = np.polyfit(ys, xs, 1)
    y_bot, y_top = h, int(h * ROI_TOP)
    return (int(m * y_bot + b), y_bot, int(m * y_top + b), y_top)


def detect_lane(frame, state=None):
    """Detect lane/path lines, draw overlay in place, return a result dict.

    Returns {'offset': float|None, 'left': line|None, 'right': line|None}.
    'offset' is the steering error in [-1, 1], or None if no path found.
    Pass the same `state` dict back in each call to enable smoothing.
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, CANNY_LO, CANNY_HI)

    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, _roi_vertices(w, h), 255)
    edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, HOUGH_THRESH,
                            minLineLength=HOUGH_MIN_LEN, maxLineGap=HOUGH_MAX_GAP)

    left, right = [], []
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < MIN_ABS_SLOPE:
                continue
            (left if slope < 0 else right).append((x1, y1, x2, y2))

    left_line = _avg_line(left, w, h)
    right_line = _avg_line(right, w, h)

    # Lane center at the bottom of the frame (closest to the car).
    lane_x = None
    if left_line and right_line:
        lane_x = (left_line[0] + right_line[0]) / 2.0
    elif left_line:                      # only left seen: assume lane to its right
        lane_x = left_line[0] + w * 0.25
    elif right_line:
        lane_x = right_line[0] - w * 0.25

    offset = None
    if lane_x is not None:
        offset = float(np.clip((lane_x - w / 2.0) / (w / 2.0), -1.0, 1.0))
        if state is not None and SMOOTH > 0 and state.get("offset") is not None:
            offset = SMOOTH * state["offset"] + (1 - SMOOTH) * offset
    if state is not None:
        state["offset"] = offset

    # --- overlay ---
    for line, col in ((left_line, (0, 255, 0)), (right_line, (0, 255, 0))):
        if line:
            cv2.line(frame, (line[0], line[1]), (line[2], line[3]), col, 3,
                     cv2.LINE_AA)
    cv2.line(frame, (w // 2, h), (w // 2, int(h * ROI_TOP)), (200, 200, 200), 1,
             cv2.LINE_AA)                                  # camera center
    if lane_x is not None:
        lx = int(lane_x)
        cv2.line(frame, (lx, h), (lx, int(h * ROI_TOP)), (0, 165, 255), 2,
                 cv2.LINE_AA)                              # lane center
        cv2.circle(frame, (lx, h - 6), 6, (0, 165, 255), -1, cv2.LINE_AA)
        # Steering bar.
        bx, by, bw = w // 2, h - 30, int(w * 0.30)
        cv2.line(frame, (bx - bw, by), (bx + bw, by), (120, 120, 120), 2)
        px = int(bx + offset * bw)
        cv2.circle(frame, (px, by), 7, (0, 165, 255), -1, cv2.LINE_AA)
        txt = f"steer: {offset:+.2f}"
    else:
        txt = "steer: ---- (no path)"
    cv2.putText(frame, txt, (12, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, txt, (12, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 165, 255), 1, cv2.LINE_AA)

    return {"offset": offset, "left": left_line, "right": right_line}


def main():
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        if img is None:
            sys.exit(f"could not read {sys.argv[1]}")
        r = detect_lane(img)
        out = "lane_" + sys.argv[1].rsplit("/", 1)[-1]
        cv2.imwrite(out, img)
        print(f"offset={r['offset']}  saved {out}")
        return
    # Live mode using the threaded camera.
    from threaded_camera import ThreadedCamera
    cam = ThreadedCamera().start()
    state, seq = {}, 0
    cv2.namedWindow("Lane", cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            frame, seq = cam.read(wait=True, last_seq=seq)
            if frame is None:
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
                continue
            frame = frame.copy()
            detect_lane(frame, state)
            cv2.imshow("Lane", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
