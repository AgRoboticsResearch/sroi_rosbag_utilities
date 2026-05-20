#!/usr/bin/env python3
"""
Batch create ORB-SLAM YAML configs for all episodes in a session (D405).

Creates the YAML from the first episode's camera info, then copies it
to all other episodes in the session directory.

Usage:
    python create_orb_slam_yaml_batch_d405.py ./recordings/1777549192/
    python create_orb_slam_yaml_batch_d405.py ./recordings/1777549192/ --skip-existing
"""

import argparse
import shutil
import sys
from pathlib import Path

from create_orb_slam_yaml import load_camera_info, create_orb_slam_yaml


def main():
    parser = argparse.ArgumentParser(
        description="Batch create ORB-SLAM YAML configs for D405 episodes"
    )
    parser.add_argument("session_dir", help="Session directory containing episode_* folders")
    parser.add_argument("--skip-existing", action="store_true",
                        help="Skip episodes that already have the YAML file")
    args = parser.parse_args()

    session_dir = Path(args.session_dir)
    if not session_dir.is_dir():
        print(f"Error: {session_dir} is not a directory")
        sys.exit(1)

    # Find episode directories
    episodes = sorted(session_dir.glob("episode_*"))
    if not episodes:
        print(f"No episode_* folders found in {session_dir}")
        sys.exit(1)

    print(f"Found {len(episodes)} episode(s) in {session_dir}")

    # Generate YAML from first episode
    first_ep = episodes[0]
    yaml_name = "orb_slam_realsense_d405.yaml"
    first_yaml = first_ep / yaml_name

    if first_yaml.exists() and args.skip_existing:
        print(f"Using existing {yaml_name} from {first_ep.name}")
    else:
        print(f"Generating {yaml_name} from {first_ep.name}...")
        template_dir = Path(__file__).parent / "orb_slam_yaml"
        template_path = template_dir / "RealSense_D405.yaml"
        camera_info = load_camera_info(str(first_ep))
        create_orb_slam_yaml(camera_info, str(template_path), str(first_yaml))

    # Copy to remaining episodes
    copied = 0
    skipped = 0
    for ep in episodes[1:]:
        target = ep / yaml_name
        if target.exists() and args.skip_existing:
            skipped += 1
            continue
        shutil.copy2(first_yaml, target)
        copied += 1
        print(f"  Copied to {ep.name}/")

    print(f"\nDone: {copied} copied, {skipped} skipped, {len(episodes)} total")


if __name__ == "__main__":
    main()
