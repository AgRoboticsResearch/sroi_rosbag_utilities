#!/usr/bin/env python3
"""Scheme 3 mask: a fixed gripper trapezoid per stereo eye plus a black bottom strip.
Accepts a schemes/session directory or one episode directory. Reads from raw/ when
present, otherwise from the episode root, and writes to episode/maskgripper/.
"""
import sys, os
from pathlib import Path
import cv2
import numpy as np

# Vertex order: top-left -> top-right -> bottom-right -> bottom-left (convex)
LEFT_POLY  = np.array([[(236, 292), (417, 292), (525, 467), (127, 467)]], dtype=np.int32)
RIGHT_POLY = np.array([[(182, 292), (365, 292), (444, 467), ( 46, 467)]], dtype=np.int32)
BOTTOM_ROW = 467  # Black out this row and everything below it


def apply_mask(img, poly):
    if img is None:
        raise ValueError("cannot mask an unreadable image")
    out = img.copy()
    cv2.fillPoly(out, poly, 0)     # Black out the gripper trapezoid
    out[BOTTOM_ROW:] = 0           # Black out the full-width bottom strip
    return out


input_path = Path(sys.argv[1]).resolve()
if not input_path.is_dir():
    raise SystemExit(f"directory not found: {input_path}")
if input_path.name.startswith("episode_"):
    eps = [input_path]
else:
    eps = sorted(d for d in input_path.iterdir() if d.is_dir() and d.name.startswith("episode_"))
if not eps:
    raise SystemExit(f"no episode_* directories found under: {input_path}")
for ep in eps:
    raw = ep / "raw" if (ep / "raw").is_dir() else ep
    mg = ep / "maskgripper"
    mg.mkdir(exist_ok=True)
    shared = [ep / "times.txt", *sorted(ep.glob("orb_slam_*.yaml"))]
    for source in shared:
        link = mg / source.name
        if source.exists() and not link.exists():
            os.symlink(f"../{source.name}", link)
    left = sorted(raw.glob("left_*.png"))
    right = sorted(raw.glob("right_*.png"))
    if not left or not right:
        raise RuntimeError(f"missing stereo PNG images in {raw}")
    for path, poly in [(p, LEFT_POLY) for p in left] + [(p, RIGHT_POLY) for p in right]:
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError(f"failed to read image: {path}")
        output = mg / path.name
        if not cv2.imwrite(str(output), apply_mask(image, poly)):
            raise RuntimeError(f"failed to write image: {output}")
    nl, nr = len(left), len(right)
    print(f"  {ep.name}: {nl}L {nr}R", flush=True)
print(f"done -> {input_path} ({len(eps)} episodes)")
