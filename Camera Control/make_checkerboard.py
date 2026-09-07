#!/usr/bin/env python3
"""
make_checkerboard.py — generate a true-scale checkerboard PDF for camera calibration.

Defaults: A4 landscape, 9x6 INNER corners (10x7 squares), 25 mm squares.

Print the PDF at 100% / "Actual size" (NOT "Fit to page"). Then verify with a
ruler against the printed 50 mm scale bar before calibrating. Tape it flat onto
something rigid (clipboard, foam board, stiff cardboard) — any bend ruins the
calibration. Measure one square with a ruler and pass that exact value as
--square-mm to calibrate_camera.py (printers scale slightly).
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# --- Geometry (millimetres) ------------------------------------------------- #
INNER_COLS, INNER_ROWS = 9, 6        # OpenCV inner-corner count
SQUARES_X, SQUARES_Y = INNER_COLS + 1, INNER_ROWS + 1   # 10 x 7 squares
SQUARE_MM = 25.0
PAGE_W, PAGE_H = 297.0, 210.0        # A4 landscape
OUT = "checkerboard_9x6_25mm.pdf"

board_w = SQUARES_X * SQUARE_MM
board_h = SQUARES_Y * SQUARE_MM
assert board_w <= PAGE_W and board_h <= PAGE_H, "board does not fit the page"

# Center the board on the page.
x0 = (PAGE_W - board_w) / 2.0
y0 = (PAGE_H - board_h) / 2.0

# Figure sized to the exact physical page so PDF maps 1 data-unit = 1 mm.
fig = plt.figure(figsize=(PAGE_W / 25.4, PAGE_H / 25.4))
ax = fig.add_axes([0, 0, 1, 1])      # full-bleed axes, no margins
ax.set_xlim(0, PAGE_W)
ax.set_ylim(0, PAGE_H)
ax.axis("off")

# Checkerboard squares (black where (i+j) is even so the outer ring is black).
for j in range(SQUARES_Y):
    for i in range(SQUARES_X):
        if (i + j) % 2 == 0:
            ax.add_patch(Rectangle((x0 + i * SQUARE_MM, y0 + j * SQUARE_MM),
                                   SQUARE_MM, SQUARE_MM,
                                   facecolor="black", edgecolor="none"))

# Thin border around the board so the printable area is obvious.
ax.add_patch(Rectangle((x0, y0), board_w, board_h,
                       fill=False, edgecolor="black", linewidth=0.5))

# Caption.
ax.text(PAGE_W / 2, y0 - 6,
        f"{INNER_COLS}x{INNER_ROWS} inner corners | {SQUARE_MM:.0f} mm squares "
        f"| PRINT AT 100% (Actual size, no scaling)",
        ha="center", va="top", fontsize=9)

# 50 mm reference scale bar (verify with a ruler after printing).
bar = 50.0
bx = x0
by = y0 + board_h + 6
ax.plot([bx, bx + bar], [by, by], color="black", linewidth=1.5)
for tick in (bx, bx + bar):
    ax.plot([tick, tick], [by - 1.5, by + 1.5], color="black", linewidth=1.5)
ax.text(bx + bar / 2, by + 3, "50 mm — measure me", ha="center", va="bottom",
        fontsize=8)

fig.savefig(OUT)
print(f"Wrote {OUT}  ({PAGE_W:.0f}x{PAGE_H:.0f} mm, "
      f"{SQUARES_X}x{SQUARES_Y} squares @ {SQUARE_MM:.0f} mm, "
      f"{INNER_COLS}x{INNER_ROWS} inner corners)")
