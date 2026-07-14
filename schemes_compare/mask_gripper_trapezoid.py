#!/usr/bin/env python3
"""方案3 掩膜：夹爪梯形(整段固定，左右红外各一套顶点) + 底部467行及以下全黑。
读 <schemes>/episode_NNN/raw/{left_,right_} 原图 → 写 maskgripper/{left_,right_}。
用法: python3 mask_gripper_trapezoid.py <schemes_dir>
"""
import sys, os
from pathlib import Path
import cv2
import numpy as np

# 顺序: 顶左 → 顶右 → 底右 → 底左 (凸梯形, 不自交)
LEFT_POLY  = np.array([[(236, 292), (417, 292), (525, 467), (127, 467)]], dtype=np.int32)
RIGHT_POLY = np.array([[(182, 292), (365, 292), (444, 467), ( 46, 467)]], dtype=np.int32)
BOTTOM_ROW = 467  # 该行及以下全部涂黑


def apply_mask(img, poly):
    out = img.copy()
    cv2.fillPoly(out, poly, 0)     # 梯形涂黑
    out[BOTTOM_ROW:] = 0           # 底部条带全宽涂黑
    return out


schemes = Path(sys.argv[1])
eps = sorted(d for d in schemes.iterdir() if d.is_dir() and d.name.startswith("episode_"))
for ep in eps:
    raw = ep / "raw"
    mg = ep / "maskgripper"
    mg.mkdir(exist_ok=True)
    for s in ["times.txt", "orb_slam_realsense_d405.yaml"]:   # symlink 共享文件
        link = mg / s
        if not link.exists():
            os.symlink(f"../{s}", link)
    nl = nr = 0
    for p in sorted(raw.glob("left_*.png")):
        cv2.imwrite(str(mg / p.name), apply_mask(cv2.imread(str(p)), LEFT_POLY));  nl += 1
    for p in sorted(raw.glob("right_*.png")):
        cv2.imwrite(str(mg / p.name), apply_mask(cv2.imread(str(p)), RIGHT_POLY)); nr += 1
    print(f"  {ep.name}: {nl}L {nr}R", flush=True)
print(f"done → {schemes} ({len(eps)} episodes)")
