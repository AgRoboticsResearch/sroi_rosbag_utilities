#!/usr/bin/env python3

"""
Convert SROI robot trajectory data to LeRobot Dataset format.

This script converts end-effector trajectory data, gripper distances, and camera images
from SROI format to a LeRobot dataset that can be used for training policies.

Usage:
    python sroi_to_lerobot.py --data_path /path/to/data --repo_id username/dataset_name
"""

import argparse
import json
import logging
import multiprocessing as mp
import numpy as np
import os
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from PIL import Image
from typing import Dict, Any

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.rotation import Rotation


def load_trajectory_data(traj_path: str):
    """
    Load and process trajectory data from CameraTrajectoryTransformed.txt
    
    Args:
        traj_path: Path to trajectory file
        
    Returns:
        timestamps: Array of timestamps
        poses: Array of 4x4 transformation matrices
        positions: Array of positions (x, y, z)
        rotvecs: Array of rotation vectors (wx, wy, wz)
    """
    # Load trajectory data, skipping comment lines
    traj = np.loadtxt(traj_path, delimiter=" ")
    print("Original trajectory shape:", traj.shape)
    
    timestamps = traj[:, 0]
    traj = traj[:, 1:]
    traj = traj.reshape(-1, 3, 4)

    # Append [0 0 0 1] to each 3x4 matrix to make it 4x4
    append_row = np.zeros((traj.shape[0], 1, 4))
    append_row[:, 0, 3] = 1
    poses = np.concatenate((traj, append_row), axis=1)
    
    # Extract positions and orientations
    positions = poses[:, :3, 3]
    rotvecs = np.array([Rotation.from_matrix(p[:3, :3]).as_rotvec() for p in poses])
    
    return timestamps, poses, positions, rotvecs


def load_gripper_data(gripper_path: str):
    """
    Load gripper distance data.
    
    Args:
        gripper_path: Path to gripper_distances.txt
        
    Returns:
        gripper_distances: Array of gripper distances
    """
    gripper_distances = np.loadtxt(gripper_path)
    return gripper_distances


def _load_image(path_str: str) -> np.ndarray:
    """Load a single image from path string. Used by process pool workers."""
    return np.array(Image.open(path_str))


def load_images_parallel(image_files, num_workers: int) -> list[np.ndarray]:
    """Load images in parallel using a process pool."""
    paths = [str(f) for f in image_files]
    if num_workers <= 1:
        return [_load_image(p) for p in paths]
    with ProcessPoolExecutor(max_workers=num_workers) as executor:
        return list(executor.map(_load_image, paths, chunksize=max(1, len(paths) // (num_workers * 4))))


def create_lerobot_dataset(
    data_path: str,
    repo_id: str,
    fps: int = 30,
    root: str = None,
    push_to_hub: bool = False,
    single_task: str = "End-effector manipulation task",
    episodes: str = "all",
    num_workers: int = 4,
):
    """
    Convert SROI data to LeRobot dataset format.

    Args:
        data_path: Path to SROI data directory (parent of episode directories)
        repo_id: Repository ID for the dataset
        fps: Frames per second
        root: Root directory for dataset storage
        push_to_hub: Whether to push to Hugging Face Hub
        single_task: Task description
        episodes: Comma-separated list of episode subdirectory names, or "all"
    """
    data_path = Path(data_path)

    # Check if data_path is a single episode directory (backward compatibility)
    single_episode_files = [
        data_path / "CameraTrajectoryTransformed.txt",
        data_path / "gripper_distances.txt",
    ]
    if all(f.exists() for f in single_episode_files):
        # Treat as single episode (backward compatible)
        episode_dirs = [data_path]
        print(f"Processing single episode: {data_path.name}")
    else:
        # Find episode directories
        if episodes == "all":
            episode_dirs = sorted([d for d in data_path.iterdir() if d.is_dir()])
        else:
            episode_names = [e.strip() for e in episodes.split(",")]
            episode_dirs = [data_path / e for e in episode_names]
            # Validate all requested episodes exist
            for ep_dir in episode_dirs:
                if not ep_dir.is_dir():
                    raise ValueError(f"Episode directory not found: {ep_dir}")

        if not episode_dirs:
            raise ValueError(f"No episode directories found in {data_path}")

        print(f"Processing {len(episode_dirs)} episode(s): {[d.name for d in episode_dirs]}")

    # Process the first episode to determine image shape for features
    first_ep_path = episode_dirs[0]
    sample_images = sorted([*first_ep_path.glob("color_*.png"), *first_ep_path.glob("color_*.jpg")])
    if not sample_images:
        raise ValueError(f"No images found in {first_ep_path}")
    sample_image = np.array(Image.open(str(sample_images[0])))
    h, w = sample_image.shape[:2]

    # Define dataset features
    # observation.state is omitted — it's derived from the action column
    # during training via DeriveStateFromActionStep (Pattern B).
    features = {
        "observation.images.camera": {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        },
        "action": {
            "dtype": "float32",
            "names": [
                "ee.x",
                "ee.y",
                "ee.z",
                "ee.wx",
                "ee.wy",
                "ee.wz",
                "ee.gripper_pos"
            ],
            "shape": (7,)
        },
    }

    # Create dataset
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        root=root,
        robot_type="so100",
        features=features,
        use_videos=True,
        image_writer_threads=1,
        image_writer_processes=1 ,
    )

    # Process each episode
    for ep_idx, ep_path in enumerate(episode_dirs):
        print(f"\n--- Processing episode {ep_idx + 1}/{len(episode_dirs)}: {ep_path.name} ---")

        # Load trajectory data
        traj_path = ep_path / "CameraTrajectoryTransformed.txt"
        if not traj_path.exists():
            print(f"Warning: Skipping {ep_path.name} - no trajectory file found")
            continue
        timestamps, poses, positions, rotvecs = load_trajectory_data(str(traj_path))

        # Load gripper data
        gripper_path = ep_path / "gripper_distances.txt"
        if not gripper_path.exists():
            print(f"Warning: Skipping {ep_path.name} - no gripper file found")
            continue
        gripper_distances = load_gripper_data(str(gripper_path))

        # Ensure all data has the same length
        min_length = min(len(timestamps), len(gripper_distances))
        timestamps = timestamps[:min_length]
        positions = positions[:min_length]
        rotvecs = rotvecs[:min_length]
        gripper_distances = gripper_distances[:min_length]

        print(f"Episode length: {min_length} frames")

        # Check for image files
        image_files = sorted([*ep_path.glob("color_*.png"), *ep_path.glob("color_*.jpg")])
        if len(image_files) < min_length:
            print(f"Warning: Only {len(image_files)} images found, but {min_length} frames needed")
            min_length = min(min_length, len(image_files))
            # Truncate other arrays
            timestamps = timestamps[:min_length]
            positions = positions[:min_length]
            rotvecs = rotvecs[:min_length]
            gripper_distances = gripper_distances[:min_length]

        print(f"Loading {min_length} images with {num_workers} worker(s)...")

        images = load_images_parallel(image_files[:min_length], num_workers)

        print("Adding frames to dataset...")

        # Add frames to dataset
        for i in range(min_length):
            image = images[i]

            # Action (next state)
            # For the last frame, we repeat the last state
            next_idx = min(i + 1, min_length - 1)
            action = np.array([
                positions[next_idx, 0],  # x
                positions[next_idx, 1],  # y
                positions[next_idx, 2],  # z
                rotvecs[next_idx, 0],    # wx
                rotvecs[next_idx, 1],    # wy
                rotvecs[next_idx, 2],    # wz
                gripper_distances[next_idx],  # gripper_pos
            ], dtype=np.float32)

            # Build frame
            frame = {
                "observation.images.camera": image,
                "action": action,
                "task": single_task,
            }

            dataset.add_frame(frame)

            if (i + 1) % 10 == 0:
                print(f"  Processed {i + 1}/{min_length} frames")

        # Save episode
        dataset.save_episode()
        print(f"Episode {ep_path.name} saved")

        # Copy camera info files per session
        camera_info_files = sorted(ep_path.glob("camera_info_*.json"))
        if camera_info_files:
            cam_dir = dataset.root / "meta" / "camera_info" / ep_path.name
            cam_dir.mkdir(parents=True, exist_ok=True)
            for cif in camera_info_files:
                shutil.copy2(cif, cam_dir / cif.name)
            print(f"Copied {len(camera_info_files)} camera info file(s) to {cam_dir}")

    print(f"\nDataset created with {dataset.num_episodes} episode(s)")
    print(f"Dataset features: {list(dataset.features.keys())}")

    dataset.finalize()

    # Push to hub if requested
    if push_to_hub:
        print("Pushing dataset to Hugging Face Hub...")
        dataset.push_to_hub()
        print("Dataset pushed successfully!")

    return dataset


def main():
    parser = argparse.ArgumentParser(description="Convert SROI data to LeRobot dataset")
    parser.add_argument(
        "--data_path",
        type=str,
        required=True,
        help="Path to SROI data directory (parent of episode directories, or a single episode directory)"
    )
    parser.add_argument(
        "--repo_id", 
        type=str, 
        required=True,
        help="Repository ID for the dataset (e.g., 'username/dataset_name')"
    )
    parser.add_argument(
        "--fps", 
        type=int, 
        default=30,
        help="Frames per second for the dataset (default: 30)"
    )
    parser.add_argument(
        "--root", 
        type=str, 
        default=None,
        help="Root directory for dataset storage (default: current directory)"
    )
    parser.add_argument(
        "--push_to_hub", 
        action="store_true",
        help="Push dataset to Hugging Face Hub"
    )
    parser.add_argument(
        "--task",
        type=str,
        default="End-effector manipulation task",
        help="Task description for the dataset"
    )
    parser.add_argument(
        "--episodes",
        type=str,
        default="all",
        help="Comma-separated list of episode subdirectory names to convert, or 'all' (default: all)"
    )
    parser.add_argument(
        "--num_workers",
        type=int,
        default=4,
        help="Number of parallel workers for image loading (default: 4, use 1 for sequential)"
    )

    args = parser.parse_args()
    
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    
    # Validate data path
    if not os.path.exists(args.data_path):
        raise ValueError(f"Data path does not exist: {args.data_path}")
    
    # Create dataset
    dataset = create_lerobot_dataset(
        data_path=args.data_path,
        repo_id=args.repo_id,
        fps=args.fps,
        root=args.root,
        push_to_hub=args.push_to_hub,
        single_task=args.task,
        episodes=args.episodes,
        num_workers=args.num_workers,
    )
    
    print("Conversion completed successfully!")
    return dataset


if __name__ == "__main__":
    main()
