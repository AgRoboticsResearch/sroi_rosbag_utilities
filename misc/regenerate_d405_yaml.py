#!/usr/bin/env python3
"""
One-off repair: regenerate every orb_slam_realsense_d405.yaml in a session tree
in proper OpenCV-YAML format (%YAML:1.0 header, !!opencv-matrix tag, flow-style
data array) using the (post-fix) camera_info_left.json + camera_info_right.json
in the same episode directory.

Needed because fix_d405_baseline.py's first version used yaml.dump() which
stripped the !!opencv-matrix tag and rewrote data in block style. ORB-SLAM3's
OpenCV FileStorage needs the tag.

Usage:
    python regenerate_d405_yaml.py /mnt/data1/sroi/sroi_v2/sroiv2_strawberry_picking_lab/20260611
"""

import json
import sys
from pathlib import Path

import numpy as np
import yaml


def emit_opencv_yaml(config: dict, output_path: Path):
    with output_path.open("w") as f:
        f.write("%YAML:1.0\n\n")
        for key, value in config.items():
            if key == "Stereo.T_c1_c2" and isinstance(value, dict):
                f.write(f"{key}: !!opencv-matrix\n")
                f.write(f"  rows: {value['rows']}\n")
                f.write(f"  cols: {value['cols']}\n")
                f.write(f"  dt: {value['dt']}\n")
                f.write(f"  data: {value['data']}\n")
            elif isinstance(value, str):
                f.write(f'{key}: "{value}"\n')
            else:
                f.write(f"{key}: {value}\n")
        f.write("\n")


def load_template(template_path: Path) -> dict:
    text = template_path.read_text()
    if text.startswith("%YAML:1.0"):
        text = text.replace("%YAML:1.0\n", "", 1)
    loader = yaml.SafeLoader
    loader.add_multi_constructor(
        "tag:yaml.org,2002:opencv-",
        lambda _l, _t, node: _l.construct_mapping(node, deep=True),
    )
    return yaml.load(text, Loader=loader)


def build_config_from_camera_info(left: dict, right: dict, template: dict) -> dict:
    cfg = dict(template)  # shallow copy
    left_K = np.array(left["K"]).reshape(3, 3)
    left_P = np.array(left["P"]).reshape(3, 4)
    right_P = np.array(right["P"]).reshape(3, 4)
    fx, fy = float(left_K[0, 0]), float(left_K[1, 1])
    cx, cy = float(left_K[0, 2]), float(left_K[1, 2])
    D = left["D"]
    for prefix in ("Camera1.", "Camera2."):
        cfg[prefix + "fx"] = fx
        cfg[prefix + "fy"] = fy
        cfg[prefix + "cx"] = cx
        cfg[prefix + "cy"] = cy
        cfg[prefix + "k1"] = float(D[0]) if len(D) > 0 else 0.0
        cfg[prefix + "k2"] = float(D[1]) if len(D) > 1 else 0.0
        cfg[prefix + "p1"] = float(D[2]) if len(D) > 2 else 0.0
        cfg[prefix + "p2"] = float(D[3]) if len(D) > 3 else 0.0
    cfg["Camera.width"] = int(left["width"])
    cfg["Camera.height"] = int(left["height"])
    baseline = float(-(right_P[0, 3] - left_P[0, 3]) / left_P[0, 0])
    T = np.eye(4, dtype=np.float32)
    T[0, 3] = baseline
    cfg["Stereo.T_c1_c2"] = {
        "rows": 4, "cols": 4, "dt": "f",
        "data": [float(x) for x in T.flatten().tolist()],
    }
    return cfg


def main():
    if len(sys.argv) != 2:
        print("Usage: python regenerate_d405_yaml.py <root>", file=sys.stderr)
        return 1
    root = Path(sys.argv[1])
    template_path = Path(__file__).resolve().parent.parent / "orb_slam_yaml" / "RealSense_D405.yaml"
    template = load_template(template_path)

    fixed = 0
    skipped = 0
    for episode in sorted(root.glob("**/episode_*")):
        if not episode.is_dir():
            continue
        left_path = episode / "camera_info_left.json"
        right_path = episode / "camera_info_right.json"
        yml_path = episode / "orb_slam_realsense_d405.yaml"
        if not (left_path.exists() and right_path.exists() and yml_path.exists()):
            skipped += 1
            continue
        left = json.loads(left_path.read_text())
        right = json.loads(right_path.read_text())
        cfg = build_config_from_camera_info(left, right, template)
        emit_opencv_yaml(cfg, yml_path)
        fixed += 1

    print(f"Regenerated: {fixed}, skipped: {skipped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
