#!/usr/bin/env python3

"""
Convert SROI robot trajectory data to LeRobot Dataset format.

This script converts end-effector trajectory data, gripper distances, and camera images
from SROI format to a LeRobot dataset that can be used for training policies.

Usage:
    python sroi_to_lerobot.py --data_path /path/to/data --repo_id username/dataset_name
"""

import argparse
import logging
import numpy as np
import os
import sys
from pathlib import Path
from PIL import Image
from typing import Dict, Any

# Ensure we're using the right paths
current_dir = os.getcwd()
if 'lerobot' not in current_dir:
    os.chdir('/home/zfei/codes/lerobot')

# Add lerobot to path
sys.path.insert(0, '/home/zfei/codes/lerobot/src')

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def euler_from_rotation_matrix(R):
    """
    Extract Euler angles (roll, pitch, yaw) from rotation matrix.
    Uses ZYX convention (yaw, pitch, roll).
    
    Args:
        R: 3x3 rotation matrix
        
    Returns:
        roll, pitch, yaw in radians
    """
    # Extract roll, pitch, yaw from rotation matrix
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
        
    return roll, pitch, yaw


def load_trajectory_data(traj_path: str):
    """
    Load and process trajectory data from CameraTrajectoryTransformed.txt
    
    Args:
        traj_path: Path to trajectory file
        
    Returns:
        timestamps: Array of timestamps
        poses: Array of 4x4 transformation matrices
        positions: Array of positions (x, y, z)
        orientations: Array of orientations (roll, pitch, yaw)
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
    positions = poses[:, :3, 3]  # Translation part
    orientations = np.zeros((len(poses), 3))  # roll, pitch, yaw
    
    for i, pose in enumerate(poses):
        rotation_matrix = pose[:3, :3]
        roll, pitch, yaw = euler_from_rotation_matrix(rotation_matrix)
        orientations[i] = [roll, pitch, yaw]
    
    return timestamps, poses, positions, orientations


def compute_deltas(positions, orientations):
    """
    Compute delta movements between consecutive frames.
    
    Args:
        positions: Array of positions (N, 3)
        orientations: Array of orientations (N, 3)
        
    Returns:
        delta_positions: Array of position deltas (N-1, 3)
        delta_orientations: Array of orientation deltas (N-1, 3)
    """
    delta_positions = np.diff(positions, axis=0)
    delta_orientations = np.diff(orientations, axis=0)
    
    # Handle angle wrapping for orientation deltas
    delta_orientations = np.where(delta_orientations > np.pi, 
                                 delta_orientations - 2*np.pi, 
                                 delta_orientations)
    delta_orientations = np.where(delta_orientations < -np.pi, 
                                 delta_orientations + 2*np.pi, 
                                 delta_orientations)
    
    return delta_positions, delta_orientations


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


def load_image(image_path: str) -> np.ndarray:
    """
    Load and process an image.
    
    Args:
        image_path: Path to image file
        
    Returns:
        image: Image array in CHW format (channels, height, width)
    """
    image = Image.open(image_path)
    image = np.array(image)
    
    # Convert to CHW format (channels, height, width)
    if len(image.shape) == 3:
        image = image.transpose(2, 0, 1)
    
    return image


def create_lerobot_dataset(
    data_path: str,
    repo_id: str,
    fps: int = 30,
    root: str = None,
    push_to_hub: bool = False,
    single_task: str = "End-effector manipulation task"
):
    """
    Convert SROI data to LeRobot dataset format.
    
    Args:
        data_path: Path to SROI data directory
        repo_id: Repository ID for the dataset
        fps: Frames per second
        root: Root directory for dataset storage
        push_to_hub: Whether to push to Hugging Face Hub
        single_task: Task description
    """
    data_path = Path(data_path)
    
    # Load trajectory data
    traj_path = data_path / "CameraTrajectoryTransformed.txt"
    timestamps, poses, positions, orientations = load_trajectory_data(str(traj_path))
    
    # Load gripper data
    gripper_path = data_path / "gripper_distances.txt"
    gripper_distances = load_gripper_data(str(gripper_path))
    
    # Ensure all data has the same length
    min_length = min(len(timestamps), len(gripper_distances))
    timestamps = timestamps[:min_length]
    positions = positions[:min_length]
    orientations = orientations[:min_length]
    gripper_distances = gripper_distances[:min_length]
    
    print(f"Dataset length: {min_length} frames")
    
    # Compute deltas for actions
    delta_positions, delta_orientations = compute_deltas(positions, orientations)
    
    # For the first frame, use zero deltas
    delta_positions = np.vstack([np.zeros(3), delta_positions])
    delta_orientations = np.vstack([np.zeros(3), delta_orientations])
    
    # Check for image files
    image_files = sorted(list(data_path.glob("color_*.png")))
    if len(image_files) < min_length:
        print(f"Warning: Only {len(image_files)} images found, but {min_length} frames needed")
        min_length = min(min_length, len(image_files))
    
    # Load a sample image to get dimensions
    sample_image = load_image(str(image_files[0]))
    image_shape = sample_image.shape
    
    # Define action names
    action_names = ["delta_x_ee", "delta_y_ee", "delta_z_ee", 
                   "delta_roll_ee", "delta_pitch_ee", "delta_yaw_ee"]
    
    # Define dataset features
    features = {
        "observation.images.camera": {
            "dtype": "video",
            "shape": image_shape,  # (channels, height, width)
            "names": ["channels", "height", "width"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (7,),  # x, y, z, roll, pitch, yaw, gripper_distance
            "names": ["x", "y", "z", "roll", "pitch", "yaw", "gripper_distance"],
        },
        "action": {
            "dtype": "float32",
            "shape": (6,),  # delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw
            "names": action_names,
        },
    }
    
    # Create dataset
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        root=root,
        robot_type="custom_ee_robot",
        features=features,
        use_videos=True,
        image_writer_threads=4,
        image_writer_processes=0,
    )
    
    print("Adding frames to dataset...")
    
    # Add frames to dataset
    for i in range(min_length):
        # Load image
        image = load_image(str(image_files[i]))
        
        # Prepare observation
        observation = {
            "images": {
                "camera": image
            },
            "state": np.array([
                positions[i, 0],  # x
                positions[i, 1],  # y  
                positions[i, 2],  # z
                orientations[i, 0],  # roll
                orientations[i, 1],  # pitch
                orientations[i, 2],  # yaw
                gripper_distances[i],  # gripper_distance
            ], dtype=np.float32)
        }
        
        # Prepare action
        action = np.array([
            delta_positions[i, 0],  # delta_x_ee
            delta_positions[i, 1],  # delta_y_ee
            delta_positions[i, 2],  # delta_z_ee
            delta_orientations[i, 0],  # delta_roll_ee
            delta_orientations[i, 1],  # delta_pitch_ee
            delta_orientations[i, 2],  # delta_yaw_ee
        ], dtype=np.float32)
        
        # Build frame
        frame = {
            "observation.images.camera": image,
            "observation.state": observation["state"],
            "action": action,
        }
        
        dataset.add_frame(frame, task=single_task)
        
        if (i + 1) % 10 == 0:
            print(f"Processed {i + 1}/{min_length} frames")
    
    # Save episode
    dataset.save_episode()
    
    print(f"Dataset created with {min_length} frames")
    print(f"Dataset episodes: {dataset.num_episodes}")
    print(f"Dataset features: {list(dataset.features.keys())}")
    
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
        help="Path to SROI data directory containing trajectory, gripper, and image files"
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
        single_task=args.task
    )
    
    print("Conversion completed successfully!")
    return dataset


if __name__ == "__main__":
    main()
