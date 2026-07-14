#!/usr/bin/env python3
"""把整个 session 所有 episode 的图像底部涂黑。原图不动，写到 -mask<cutoff>/。

用法: python mask_session.py <png_session> <cutoff_row>
例:   python mask_session.py .../20260709_144418-png 292
      (第 292 行及以下涂黑；对应去掉底部 38/97 ≈ 39%)
"""
import sys, shutil
from pathlib import Path
import cv2


def main():
    src = Path(sys.argv[1]).resolve()
    cutoff = int(sys.argv[2])
    dst = src.parent / f"{src.name}-mask{cutoff}"
    eps = sorted(d for d in src.iterdir() if d.is_dir() and d.name.startswith("episode_"))
    print(f"{src.name}: {len(eps)} episodes, cutoff={cutoff} (第{cutoff}行及以下涂黑) → {dst.name}")
    for i, ep in enumerate(eps, 1):
        d = dst / ep.name
        d.mkdir(parents=True, exist_ok=True)
        for name in ["orb_slam_realsense_d405.yaml", "times.txt", "timestamps.json",
                     "camera_info_left.json", "camera_info_right.json", "camera_info_color.json"]:
            s = ep / name
            if s.exists():
                shutil.copy2(s, d / name)
        for pat in ["left_*.png", "right_*.png", "color_*.png"]:
            for p in sorted(ep.glob(pat)):
                img = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
                if img is None:
                    continue
                img[cutoff:, ...] = 0
                cv2.imwrite(str(d / p.name), img)
        print(f"  [{i}/{len(eps)}] {ep.name}", flush=True)
    print(f"done → {dst}")


if __name__ == "__main__":
    main()
