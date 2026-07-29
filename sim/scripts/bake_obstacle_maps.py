#!/usr/bin/env python3
"""Bake each obstacle set from config/obstacle_sets.yaml into a map variant.

The MuSHR sim's raycasting LiDAR sees only the occupancy grid, so obstacles
must be rasterized into the map to be detectable. Produces
sim/maps/track_<set>.pgm/.yaml alongside an obstacle-free copy.
"""
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
RES = 0.05
OUT = ROOT / "sim" / "maps"
OUT.mkdir(parents=True, exist_ok=True)

base = np.array(Image.open(ROOT / "track" / "mushr_training_track.pgm"))
H, W = base.shape
sets = yaml.safe_load((ROOT / "config" / "obstacle_sets.yaml").read_text())


def to_px(x, y):
    return int(round(x / RES)), int(round((H - 1) - y / RES))


def draw_box(arr, x, y, yaw, sx, sy):
    hx, hy = sx / 2, sy / 2
    c, s = math.cos(yaw), math.sin(yaw)
    # rasterize by sampling the box footprint
    step = RES / 2
    for u in np.arange(-hx, hx + step, step):
        for v in np.arange(-hy, hy + step, step):
            px, py = to_px(x + u * c - v * s, y + u * s + v * c)
            if 0 <= px < W and 0 <= py < H:
                arr[py, px] = 0


def draw_cylinder(arr, x, y, r):
    step = RES / 2
    for u in np.arange(-r, r + step, step):
        for v in np.arange(-r, r + step, step):
            if u * u + v * v <= r * r:
                px, py = to_px(x + u, y + v)
                if 0 <= px < W and 0 <= py < H:
                    arr[py, px] = 0


def write_map(name, arr):
    Image.fromarray(arr).save(OUT / f"{name}.pgm")
    (OUT / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nresolution: {RES}\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")


write_map("track_clean", base.copy())
only = sys.argv[1] if len(sys.argv) > 1 else None
for set_name, obstacles in sets.items():
    if only and set_name != only:
        continue
    arr = base.copy()
    for o in obstacles:
        if o["type"] == "cylinder":
            draw_cylinder(arr, o["x"], o["y"], o["radius"])
        else:
            draw_box(arr, o["x"], o["y"], o.get("yaw", 0.0), *o["size"])
    write_map(f"track_{set_name}", arr)
    print(f"baked track_{set_name} ({len(obstacles)} obstacles)")
print("maps written to", OUT)
