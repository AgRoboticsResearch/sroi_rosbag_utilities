#!/usr/bin/env python
"""Convert EE trajectory + RGB images + timestamps into LeRobot dataset format.

Matches the standard LeRobot EE pose convention used in phone_to_so100:
  - observation.state.ee.{x, y, z, wx, wy, wz, gripper_pos}  (individual scalar features)
  - action (composite 7-vector with named axes)
  - observation.images.{camera} (video)

Input directory structure:
  input_dir/
  ├── episode_001/
  │   ├── EETrajectory.txt    # 4x4 transforms, 12 floats per line (row-major)
  │   ├── times.txt           # Unix timestamps, one per line
  │   ├── color_000000.png    # RGB images
  │   └── ...
  └── episode_002/
      └── ...

Usage:
  cd lerobot
  uv sync --extra dataset
  uv run python ../convert_to_lerobot.py \
      --input-dir ../1777628135 \
      --repo-id my_robot_dataset \
      --task "pick and place"
"""

import argparse
import os
import numpy as np
from pathlib import Path
from PIL import Image

from lerobot.datasets import LeRobotDataset


def load_ee_trajectory(path):
    """Load 4x4 transform matrices from EETrajectory.txt (12 floats per line)."""
    poses = []
    with open(path) as f:
        for line in f:
            vals = [float(x) for x in line.strip().split()]
            if len(vals) != 12:
                continue
            T = np.eye(4)
            T[0, :4] = vals[0:4]
            T[1, :4] = vals[4:8]
            T[2, :4] = vals[8:12]
            poses.append(T)
    return poses


def load_timestamps(path):
    """Load unix timestamps, one per line."""
    with open(path) as f:
        return [float(line.strip()) for line in f if line.strip()]


def load_gripper_distances(path):
    """Load normalized gripper distance values, one per line. Returns None if file missing."""
    if not path.exists():
        return None
    with open(path) as f:
        return [float(line.strip()) for line in f if line.strip()]


def rotation_matrix_to_rotvec(R):
    """Convert 3x3 rotation matrix to a rotation vector (axis * angle).

    This is the standard LeRobot representation (ee.wx, ee.wy, ee.wz).
    """
    eps = 1e-8
    # trace = R[0,0] + R[1,1] + R[2,2]
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    if theta < eps:
        return np.zeros(3, dtype=np.float32)

    if np.abs(theta - np.pi) < eps:
        # 180-degree case: axis is eigenvector of (R+I) corresponding to non-zero columns
        R_plus_I = R + np.eye(3)
        # Find a non-zero column
        for j in range(3):
            v = R_plus_I[:, j]
            if np.linalg.norm(v) > eps:
                axis = v / np.linalg.norm(v)
                break
        else:
            return np.zeros(3, dtype=np.float32)
        return (axis * theta).astype(np.float32)

    # Standard case
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2.0 * np.sin(theta))

    return (axis * theta).astype(np.float32)


def transform_to_ee_parts(T):
    """Extract xyz position and rotation vector from a 4x4 transform.

    Returns (x, y, z, wx, wy, wz).
    """
    pos = T[:3, 3]
    tw = rotation_matrix_to_rotvec(T[:3, :3])
    return np.concatenate([pos, tw]).astype(np.float32)


def get_image_files(ep_dir, camera_name):
    """Get sorted list of image paths for a camera (e.g. color_*.png)."""
    return sorted(ep_dir.glob(f"{camera_name}_*.png"))


def get_image_size(img_path):
    """Return (height, width) of an image."""
    img = Image.open(str(img_path))
    return img.size[1], img.size[0]


def compute_fps(timestamps):
    """Estimate fixed FPS from timestamps."""
    if len(timestamps) < 2:
        return 30
    diffs = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
    avg = sum(diffs) / len(diffs)
    return round(1.0 / avg)


def discover_episodes(input_dir, recursive=False):
    """Find episode directories containing EETrajectory.txt + times.txt."""
    input_path = Path(input_dir)

    if (input_path / "EETrajectory.txt").exists() and (input_path / "times.txt").exists():
        return [input_path]

    episodes = []
    if recursive:
        for dirpath, _, filenames in os.walk(input_path):
            if "EETrajectory.txt" in filenames and "times.txt" in filenames:
                episodes.append(Path(dirpath))
    else:
        for d in sorted(input_path.glob("episode_*")):
            if (d / "EETrajectory.txt").exists() and (d / "times.txt").exists():
                episodes.append(d)
    return episodes


# EE pose axes names used in LeRobot composite action
EE_AXES = ["ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz", "ee.gripper_pos"]


def main():
    parser = argparse.ArgumentParser(
        description="Convert EE trajectory + images to LeRobot dataset"
    )
    parser.add_argument(
        "--input-dir", required=True, help="Directory containing episode subdirectories"
    )
    parser.add_argument(
        "--repo-id", required=True, help="Dataset name / repo id (e.g. my_robot_dataset)"
    )
    parser.add_argument(
        "--robot-type", default="custom", help="Robot type identifier"
    )
    parser.add_argument(
        "--cameras", nargs="+", default=["color"],
        help="Camera name prefixes to include (default: color)"
    )
    parser.add_argument(
        "--task", required=True,
        help="Task description string (stored as task feature)"
    )
    parser.add_argument(
        "--push-to-hub", action="store_true", help="Upload to Hugging Face Hub after conversion"
    )
    parser.add_argument(
        "--recursive", action="store_true",
        help="Search for episode directories recursively under input-dir"
    )
    args = parser.parse_args()

    episodes = discover_episodes(args.input_dir, recursive=args.recursive)
    if not episodes:
        print(f"ERROR: No episode directories (with EETrajectory.txt + times.txt) found in {args.input_dir}")
        print("Expected structure: input_dir/episode_XXX/{EETrajectory.txt, times.txt, *.png}")
        return

    print(f"Found {len(episodes)} episode(s)")

    # Probe image sizes from first episode
    first_ep = episodes[0]
    image_sizes = {}
    for cam in args.cameras:
        files = get_image_files(first_ep, cam)
        if files:
            h, w = get_image_size(files[0])
            image_sizes[cam] = (h, w)
            print(f"  Camera '{cam}': {w}x{h}")
        else:
            print(f"  WARNING: No images found for camera '{cam}', skipping")

    if not image_sizes:
        print("ERROR: No camera images found. At least one camera is required.")
        return

    # Estimate FPS
    fps = compute_fps(load_timestamps(first_ep / "times.txt"))
    print(f"Estimated FPS: {fps}")

    # ---- Build feature schema (composite vectors — best for policy training) ----
    features = {}

    # Composite observation state vector: [x, y, z, wx, wy, wz, gripper_pos]
    features["observation.state"] = {
        "dtype": "float32",
        "shape": (7,),
        "names": {"axes": EE_AXES},
    }

    # Camera images
    for cam in args.cameras:
        if cam not in image_sizes:
            continue
        h, w = image_sizes[cam]
        features[f"observation.images.{cam}"] = {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }

    # Composite action with named axes (like EEReferenceAndDelta output)
    features["action"] = {
        "dtype": "float32",
        "shape": (7,),
        "names": {"axes": EE_AXES},
    }

    # ---- Create dataset ----
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        robot_type=args.robot_type,
        fps=fps,
        features=features,
    )
    print(f"Dataset root: {dataset.root}")

    # ---- Process episodes ----
    for ep_idx, ep_dir in enumerate(episodes):
        print(f"\nProcessing episode {ep_idx} ({ep_dir.name}) ...")

        times = load_timestamps(ep_dir / "times.txt")
        poses = load_ee_trajectory(ep_dir / "EETrajectory.txt")
        grippers = load_gripper_distances(ep_dir / "gripper_distances.txt")
        n_frames = min(len(poses), len(times))

        if grippers:
            if len(grippers) != n_frames:
                print(f"  WARNING: {len(grippers)} gripper values vs {n_frames} frames. Using min count.")
                n_frames = min(n_frames, len(grippers))
        else:
            print(f"  No gripper_distances.txt found, setting gripper_pos = 0")

        # Verify image counts
        for cam in args.cameras:
            if cam not in image_sizes:
                continue
            img_files = get_image_files(ep_dir, cam)
            if len(img_files) != n_frames:
                print(f"  WARNING: {len(img_files)} {cam} images vs {n_frames} frames. Using min count.")
                n_frames = min(n_frames, len(img_files))

        # Pre-load images
        cam_images = {}
        for cam in args.cameras:
            if cam not in image_sizes:
                continue
            img_files = get_image_files(ep_dir, cam)[:n_frames]
            cam_images[cam] = [np.array(Image.open(str(f))) for f in img_files]

        print(f"  Converting {n_frames} frames ...")

        for i in range(n_frames):
            T = poses[i]
            ee = transform_to_ee_parts(T)  # [x, y, z, wx, wy, wz]
            gripper = float(grippers[i]) if grippers else 0.0
            state = np.array([ee[0], ee[1], ee[2], ee[3], ee[4], ee[5], gripper], dtype=np.float32)

            frame = {
                "observation.state": state,
                "action": state.copy(),  # absolute EE pose as action
                "task": args.task,
            }

            for cam in args.cameras:
                if cam not in image_sizes:
                    continue
                frame[f"observation.images.{cam}"] = cam_images[cam][i]

            dataset.add_frame(frame)

        dataset.save_episode()
        print(f"  Episode {ep_idx} saved.")

    # ---- Finalize ----
    dataset.finalize()
    print(f"\nDataset finalized: {dataset.root}")

    if args.push_to_hub:
        print("Pushing to Hugging Face Hub ...")
        dataset.push_to_hub(private=True)
        print(f"Pushed: https://huggingface.co/datasets/{args.repo_id}")


if __name__ == "__main__":
    main()
