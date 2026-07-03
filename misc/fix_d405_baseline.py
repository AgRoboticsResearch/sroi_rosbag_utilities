#!/usr/bin/env python3
"""
Post-hoc fix for D405 stereo baseline recorded 1000x too small.

pyrealsense2 returns the D405 stereo extrinsics translation in millimeters but
record_realsense.py (prior to the sanity-check fix) wrote it as meters, so every
recorded session has:
  - camera_info_right.json P[3] off by 1000x (e.g. +0.0193 instead of +19.3)
  - orb_slam_realsense_d405.yaml Stereo.T_c1_c2[0,3] off by 1000x (e.g. -5e-5 instead of -0.05)

ORB-SLAM3 cannot triangulate with such a tiny baseline and produces a trajectory
that collapses to identity. This script multiplies the offending values by 1000
in place across all sessions under a given root, so existing recordings can be
re-processed without re-recording.

Idempotent: skips files that already look correct (P[3] > 0.5 m or tx > 0.0005 m).

Usage:
    python fix_d405_baseline.py /mnt/data1/sroi/sroi_v2/sroiv2_strawberry_picking_lab/20260611
    python fix_d405_baseline.py /path/to/single/session-mp4
"""

import argparse
import json
import re
import sys
from pathlib import Path

import yaml


def load_opencv_yaml(path: Path):
    """Load OpenCV-style YAML (%YAML:1.0 header, !!opencv-matrix tags) PyYAML rejects by default."""
    text = path.read_text()
    lines = [ln for ln in text.splitlines() if not ln.lstrip().startswith("%YAML")]
    loader = yaml.SafeLoader
    # !!opencv-matrix is just a map with rows/cols/dt/data fields
    loader.add_multi_constructor(
        "tag:yaml.org,2002:opencv-",
        lambda _loader, _tag, node: _loader.construct_mapping(node, deep=True),
    )
    return yaml.load("\n".join(lines), Loader=loader)


# Threshold below which we treat the baseline as broken (in meters, on the P[3] value).
# Real D405: P[3] = -fx * baseline = -390 * 0.05 = ~-19.5 (magnitude ~19.5).
# Broken recordings show ~0.02. The midpoint 0.5 cleanly separates them.
BROKEN_P3_THRESHOLD = 0.5
SCALE = 1000.0


def fix_camera_info_right(path: Path) -> str:
    """Multiply P[3] by SCALE if it looks broken. Returns 'skipped' / 'fixed' / 'no-right'."""
    with path.open() as f:
        d = json.load(f)
    P = d.get("P")
    if not P or len(P) != 12:
        return "no-P"
    p3 = P[3]
    if abs(p3) > BROKEN_P3_THRESHOLD:
        return "skipped"
    P[3] = p3 * SCALE
    with path.open("w") as f:
        json.dump(d, f, indent=4)
    return "fixed"


def _read_tx_from_yaml(path: Path):
    """Return the Stereo.T_c1_c2[0,3] value from a yaml, or None if absent."""
    d = load_opencv_yaml(path)
    t = d.get("Stereo.T_c1_c2") if isinstance(d, dict) else None
    if not isinstance(t, dict):
        return None
    data = t.get("data")
    if not isinstance(data, list) or len(data) != 16:
        return None
    return float(data[3])


def fix_orb_slam_yaml(path: Path) -> str:
    """Multiply Stereo.T_c1_c2[0,3] by SCALE in place via regex, preserving the
    original OpenCV-YAML layout (tag + flow-style data array) that FileStorage expects."""
    tx = _read_tx_from_yaml(path)
    if tx is None:
        return "bad-data"
    if abs(tx) > BROKEN_P3_THRESHOLD / 100.0:  # yaml tx is in meters; ~0.05 expected
        return "skipped"
    new_tx = tx * SCALE
    # Match the data: [...] line under Stereo.T_c1_c2 only (it's the only 16-element
    # matrix in the file). Replace the 4th number (index 3) with the corrected value.
    text = path.read_text()
    pattern = re.compile(
        r"(Stereo\.T_c1_c2:.*?data:\s*\[)([^]]+)(\])",
        re.DOTALL,
    )
    m = pattern.search(text)
    if not m:
        return "bad-data"
    nums = [n.strip() for n in m.group(2).split(",")]
    if len(nums) != 16:
        return "bad-data"
    nums[3] = repr(new_tx)
    new_text = text[: m.start()] + m.group(1) + ", ".join(nums) + m.group(3) + text[m.end():]
    path.write_text(new_text)
    return "fixed"


def find_session_roots(root: Path):
    """Yield directories that look like a session (contain episode_* dirs)."""
    for d in sorted(root.iterdir()):
        if d.is_dir() and any((d / e).is_dir() for e in d.iterdir() if e.name.startswith("episode_")):
            yield d
        elif d.is_dir() and d.name.startswith("episode_"):
            # Caller passed a session directly
            yield d.parent
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("root", type=Path, help="Directory containing session folders (or a single session)")
    parser.add_argument("--dry-run", action="store_true", help="Report what would change without writing")
    args = parser.parse_args()

    if not args.root.exists():
        print(f"Path not found: {args.root}", file=sys.stderr)
        return 1

    sessions = sorted(set(find_session_roots(args.root)))
    if not sessions:
        print(f"No session folders (containing episode_*) found under {args.root}", file=sys.stderr)
        return 1

    print(f"Scanning {len(sessions)} session(s) under {args.root}\n")

    stats = {"camera_info_right": {"fixed": 0, "skipped": 0, "missing": 0},
             "orb_slam_yaml": {"fixed": 0, "skipped": 0, "missing": 0}}

    for session in sessions:
        for episode in sorted(session.glob("episode_*")):
            if not episode.is_dir():
                continue

            cir = episode / "camera_info_right.json"
            if cir.exists():
                if args.dry_run:
                    with cir.open() as f:
                        d = json.load(f)
                    p3 = d["P"][3]
                    action = "would fix" if abs(p3) <= BROKEN_P3_THRESHOLD else "skip"
                    print(f"  [dry-run] {cir.relative_to(args.root)}: P[3]={p3:.6g} -> {action}")
                else:
                    res = fix_camera_info_right(cir)
                    stats["camera_info_right"][res if res in stats["camera_info_right"] else "missing"] += 1
            else:
                stats["camera_info_right"]["missing"] += 1

            for yml in episode.glob("orb_slam_*.yaml"):
                if args.dry_run:
                    d = load_opencv_yaml(yml)
                    try:
                        tx = d["Stereo.T_c1_c2"]["data"][3]
                    except (KeyError, TypeError, IndexError):
                        tx = None
                    if tx is None:
                        action = "no T_c1_c2"
                    else:
                        action = "would fix" if abs(tx) <= BROKEN_P3_THRESHOLD / 100.0 else "skip"
                    print(f"  [dry-run] {yml.relative_to(args.root)}: tx={tx} -> {action}")
                else:
                    res = fix_orb_slam_yaml(yml)
                    key = res if res in stats["orb_slam_yaml"] else "missing"
                    stats["orb_slam_yaml"][key] += 1

    if not args.dry_run:
        print("camera_info_right.json:")
        for k, v in stats["camera_info_right"].items():
            print(f"  {k:8s}: {v}")
        print("orb_slam_*.yaml:")
        for k, v in stats["orb_slam_yaml"].items():
            print(f"  {k:8s}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
