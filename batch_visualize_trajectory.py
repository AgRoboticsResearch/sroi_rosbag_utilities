#!/usr/bin/env python3
"""
Batch Trajectory Visualization Script

Visualizes camera trajectories from ORB-SLAM output files.
Generates individual plots per segment and a combined grid overview.

Usage:
    python batch_visualize_trajectory.py <input_folder> [options]

Author: Generated for SROI ROS Bag Utilities
Date: 2026-03-04
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
import argparse
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing


def find_trajectory_files(input_folder):
    """
    Find all CameraTrajectory.txt files recursively.

    Args:
        input_folder (str): Root folder to search

    Returns:
        list: List of paths to CameraTrajectory.txt files
    """
    trajectory_files = []
    for root, dirs, files in os.walk(input_folder):
        if 'CameraTrajectory.txt' in files:
            trajectory_files.append(os.path.join(root, 'CameraTrajectory.txt'))
    return sorted(trajectory_files)


def load_trajectory(traj_path):
    """
    Load trajectory from file.

    Args:
        traj_path (str): Path to CameraTrajectory.txt

    Returns:
        tuple: (trajectory_3x4, segment_name, folder_path)
    """
    traj = np.loadtxt(traj_path, delimiter=' ')
    traj = traj.reshape(-1, 3, 4)
    folder_path = os.path.dirname(traj_path)
    segment_name = os.path.basename(folder_path)
    return traj, segment_name, folder_path


def compute_statistics(traj):
    """
    Compute trajectory statistics.

    Args:
        traj (np.ndarray): Trajectory as Nx3x4 matrices

    Returns:
        dict: Statistics including path length, position ranges
    """
    # Extract translations
    translations = traj[:, :3, 3]

    # Compute path length
    diffs = np.diff(translations, axis=0)
    path_length = np.sum(np.linalg.norm(diffs, axis=1))

    # Position ranges
    pos_min = translations.min(axis=0)
    pos_max = translations.max(axis=0)
    pos_range = pos_max - pos_min

    return {
        'num_poses': len(traj),
        'path_length': path_length,
        'x_range': (pos_min[0], pos_max[0]),
        'y_range': (pos_min[1], pos_max[1]),
        'z_range': (pos_min[2], pos_max[2]),
        'x_span': pos_range[0],
        'y_span': pos_range[1],
        'z_span': pos_range[2],
    }


def visualize_single_trajectory(traj, segment_name, stats, output_path, uniform_scale=None):
    """
    Create visualization for a single trajectory.

    Args:
        traj (np.ndarray): Trajectory as Nx3x4 matrices
        segment_name (str): Name of the segment
        stats (dict): Statistics dictionary
        output_path (str): Path to save the JPG
        uniform_scale (tuple): Optional (max_span,) for uniform scaling
    """
    # Extract translations
    x = traj[:, 0, 3]
    y = traj[:, 1, 3]
    z = traj[:, 2, 3]

    # Create figure
    fig = plt.figure(figsize=(12, 5))

    # 3D trajectory
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(x, z, y, 'b-', linewidth=1.5, label='Trajectory')
    ax1.scatter([x[0]], [z[0]], [y[0]], c='green', s=100, marker='o', label='Start')
    ax1.scatter([x[-1]], [z[-1]], [y[-1]], c='red', s=100, marker='x', label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Z (m)')
    ax1.set_zlabel('Y (m)')
    ax1.set_title('3D Camera Trajectory')
    ax1.legend()

    # Apply uniform scale if provided
    if uniform_scale:
        max_span = uniform_scale
        center_x = (x.max() + x.min()) / 2
        center_y = (y.max() + y.min()) / 2
        center_z = (z.max() + z.min()) / 2
        ax1.set_xlim(center_x - max_span/2, center_x + max_span/2)
        ax1.set_ylim(center_z - max_span/2, center_z + max_span/2)
        ax1.set_zlim(center_y - max_span/2, center_y + max_span/2)

    # 2D top-down view (X-Z plane)
    ax2 = fig.add_subplot(122)
    ax2.plot(x, z, 'b-', linewidth=1.5)
    ax2.scatter(x[0], z[0], c='green', s=100, marker='o', label='Start', zorder=5)
    ax2.scatter(x[-1], z[-1], c='red', s=100, marker='x', label='End', zorder=5)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('Top-Down View (X-Z)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Apply uniform scale for 2D plot
    if uniform_scale:
        ax2.set_xlim(center_x - max_span/2, center_x + max_span/2)
        ax2.set_ylim(center_z - max_span/2, center_z + max_span/2)
        ax2.set_aspect('equal')
    else:
        ax2.axis('equal')

    # Add statistics text
    stats_text = (
        f"Poses: {stats['num_poses']} | "
        f"Path: {stats['path_length']:.3f}m\n"
        f"X: [{stats['x_range'][0]:.3f}, {stats['x_range'][1]:.3f}] | "
        f"Y: [{stats['y_range'][0]:.3f}, {stats['y_range'][1]:.3f}] | "
        f"Z: [{stats['z_range'][0]:.3f}, {stats['z_range'][1]:.3f}]"
    )
    fig.suptitle(f"{segment_name}", fontsize=12, fontweight='bold')
    fig.text(0.5, 0.02, stats_text, ha='center', fontsize=9, family='monospace')

    plt.tight_layout(rect=[0, 0.08, 1, 0.95])
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()


def process_single_trajectory(args):
    """
    Process a single trajectory file (for parallel execution).

    Args:
        args (tuple): (traj_path, skip_existing, uniform_scale)

    Returns:
        tuple: (segment_name, stats, success, output_path)
    """
    traj_path, skip_existing, uniform_scale = args

    try:
        folder_path = os.path.dirname(traj_path)
        segment_name = os.path.basename(folder_path)
        output_path = os.path.join(folder_path, 'trajectory_viz.jpg')

        # Skip if exists
        if skip_existing and os.path.exists(output_path):
            return segment_name, None, None, 'skipped', output_path

        # Load and process
        traj, segment_name, folder_path = load_trajectory(traj_path)
        stats = compute_statistics(traj)
        visualize_single_trajectory(traj, segment_name, stats, output_path, uniform_scale)

        return segment_name, traj, stats, True, output_path

    except Exception as e:
        return os.path.basename(os.path.dirname(traj_path)), None, None, str(e), None


def extract_short_name(segment_name):
    """
    Extract short name from segment folder name.

    Examples:
        rs435_2026-02-06-05-39-45_segment_16 -> 05-39-45 #16
        rs435_2026-02-06-02-57-39_recovered_segment_1 -> 02-57-39 #1 (rec)
    """
    import re

    # Extract time (HH-MM-SS pattern)
    time_match = re.search(r'(\d{2}-\d{2}-\d{2})', segment_name)
    time_str = time_match.group(1) if time_match else "??-??-??"

    # Extract segment number
    seg_match = re.search(r'segment_(\d+)', segment_name)
    seg_num = seg_match.group(1) if seg_match else "?"

    # Check if recovered
    is_recovered = 'recovered' in segment_name

    short_name = f"{time_str} #{seg_num}"
    if is_recovered:
        short_name += " (rec)"

    return short_name


def create_combined_grid(trajectory_data, output_path, max_cols=5):
    """
    Create a combined grid overview of all trajectories.

    Args:
        trajectory_data (list): List of (traj, segment_name, stats) tuples
        output_path (str): Path to save the combined image
        max_cols (int): Maximum number of columns in the grid
    """
    n = len(trajectory_data)
    cols = min(n, max_cols)
    rows = (n + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(cols * 4, rows * 3))
    axes = axes.flatten() if n > 1 else [axes]

    for idx, (traj, segment_name, stats) in enumerate(trajectory_data):
        ax = axes[idx]
        x = traj[:, 0, 3]
        z = traj[:, 2, 3]

        ax.plot(x, z, 'b-', linewidth=1)
        ax.scatter(x[0], z[0], c='green', s=30, marker='o', zorder=5)
        ax.scatter(x[-1], z[-1], c='red', s=30, marker='x', zorder=5)

        # Use short name format
        short_name = extract_short_name(segment_name)
        ax.set_title(f"{short_name}\n{stats['num_poses']}p, {stats['path_length']:.2f}m",
                     fontsize=8)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

    # Hide empty subplots
    for idx in range(len(trajectory_data), len(axes)):
        axes[idx].axis('off')

    fig.suptitle('Trajectory Overview', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()


def find_gripper_files(input_folder):
    """
    Find all gripper_distances.txt files recursively.

    Args:
        input_folder (str): Root folder to search

    Returns:
        list: List of paths to gripper_distances.txt files
    """
    gripper_files = []
    for root, dirs, files in os.walk(input_folder):
        if 'gripper_distances.txt' in files:
            gripper_files.append(os.path.join(root, 'gripper_distances.txt'))
    return sorted(gripper_files)


def load_gripper_distances(gripper_path):
    """
    Load gripper distances from file.

    Args:
        gripper_path (str): Path to gripper_distances.txt

    Returns:
        tuple: (distances, segment_name, folder_path)
    """
    distances = np.loadtxt(gripper_path)
    folder_path = os.path.dirname(gripper_path)
    segment_name = os.path.basename(folder_path)
    return distances, segment_name, folder_path


def compute_gripper_stats(distances):
    """
    Compute gripper distance statistics.

    Args:
        distances (np.ndarray): Array of gripper distances

    Returns:
        dict: Statistics including min, max, mean, range
    """
    return {
        'num_samples': len(distances),
        'min': distances.min(),
        'max': distances.max(),
        'mean': distances.mean(),
        'std': distances.std(),
        'range': distances.max() - distances.min(),
    }


def create_gripper_grid(gripper_data, output_path, max_cols=5):
    """
    Create a combined grid overview of all gripper distances.

    Args:
        gripper_data (list): List of (distances, segment_name, stats) tuples
        output_path (str): Path to save the combined image
        max_cols (int): Maximum number of columns in the grid
    """
    n = len(gripper_data)
    cols = min(n, max_cols)
    rows = (n + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(cols * 4, rows * 3))
    axes = axes.flatten() if n > 1 else [axes]

    # Compute global y-axis limits for consistency
    all_distances = np.concatenate([d[0] for d in gripper_data])
    y_min = all_distances.min()
    y_max = all_distances.max()
    y_margin = (y_max - y_min) * 0.1
    y_lim = (y_min - y_margin, y_max + y_margin)

    for idx, (distances, segment_name, stats) in enumerate(gripper_data):
        ax = axes[idx]

        # Plot gripper distance over time
        ax.plot(distances, 'b-', linewidth=0.8)
        ax.axhline(y=stats['mean'], color='orange', linestyle='--', linewidth=0.5, alpha=0.7)

        # Use short name format
        short_name = extract_short_name(segment_name)
        ax.set_title(f"{short_name}\n{stats['num_samples']}s, range:{stats['range']:.3f}",
                     fontsize=8)
        ax.set_ylim(y_lim)
        ax.set_xticks([])
        ax.grid(True, alpha=0.3)

    # Hide empty subplots
    for idx in range(len(gripper_data), len(axes)):
        axes[idx].axis('off')

    fig.suptitle('Gripper Distance Overview', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Batch visualize camera trajectories",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python batch_visualize_trajectory.py test_output/
    python batch_visualize_trajectory.py test_output/ --skip --scale uniform
        """
    )

    parser.add_argument(
        'input_folder',
        type=str,
        help='Input folder containing segment subfolders with CameraTrajectory.txt'
    )

    parser.add_argument(
        '--skip',
        action='store_true',
        help='Skip trajectories that already have visualization'
    )

    parser.add_argument(
        '--scale',
        type=str,
        choices=['auto', 'uniform'],
        default='auto',
        help='Scale mode: auto (per-trajectory) or uniform (same scale for all)'
    )

    parser.add_argument(
        '--workers',
        type=int,
        default=multiprocessing.cpu_count(),
        help='Number of parallel workers (default: CPU count)'
    )

    parser.add_argument(
        '--grid-only',
        action='store_true',
        help='Only generate combined grid, skip individual plots'
    )

    args = parser.parse_args()

    # Validate input folder
    input_folder = Path(args.input_folder)
    if not input_folder.exists():
        print(f"Error: Input folder '{input_folder}' does not exist")
        sys.exit(1)

    # Find all trajectory files
    print(f"Searching for trajectories in: {input_folder}")
    trajectory_files = find_trajectory_files(str(input_folder))

    if not trajectory_files:
        print("No CameraTrajectory.txt files found")
        sys.exit(1)

    print(f"Found {len(trajectory_files)} trajectories")

    # Determine uniform scale if needed
    uniform_scale = None
    if args.scale == 'uniform':
        print("Computing uniform scale...")
        max_span = 0
        for traj_path in trajectory_files:
            traj, _, _ = load_trajectory(traj_path)
            stats = compute_statistics(traj)
            span = max(stats['x_span'], stats['y_span'], stats['z_span'])
            max_span = max(max_span, span)
        uniform_scale = max_span * 1.1  # Add 10% margin
        print(f"Uniform scale: {uniform_scale:.3f}m")

    # Process individual trajectories
    all_trajectory_data = []
    processed = 0
    skipped = 0
    failed = 0

    print("\nProcessing trajectories...")

    # Prepare arguments for parallel processing
    process_args = [(tf, args.skip, uniform_scale) for tf in trajectory_files]

    # Use sequential processing to avoid matplotlib issues
    for i, traj_path in enumerate(trajectory_files):
        segment_name, traj, stats, status, output_path = process_single_trajectory(
            (traj_path, args.skip, uniform_scale)
        )

        if status == 'skipped':
            skipped += 1
            print(f"  [{i+1}/{len(trajectory_files)}] Skipped: {segment_name}")
            # Load data for grid if needed
            if not args.grid_only:
                traj, _, _ = load_trajectory(traj_path)
                stats = compute_statistics(traj)
                all_trajectory_data.append((traj, segment_name, stats))
        elif status == True:
            processed += 1
            print(f"  [{i+1}/{len(trajectory_files)}] Generated: {segment_name}")
            all_trajectory_data.append((traj, segment_name, stats))
        else:
            failed += 1
            print(f"  [{i+1}/{len(trajectory_files)}] Failed: {segment_name} - {status}")

    # Create combined grid
    if all_trajectory_data:
        grid_path = input_folder / 'trajectory_overview.jpg'
        print(f"\nGenerating combined grid: {grid_path}")
        create_combined_grid(all_trajectory_data, str(grid_path))
        print(f"Combined grid saved!")

    # Create gripper distance grid
    print(f"\nSearching for gripper distances in: {input_folder}")
    gripper_files = find_gripper_files(str(input_folder))

    if gripper_files:
        print(f"Found {len(gripper_files)} gripper distance files")

        all_gripper_data = []
        for gripper_path in gripper_files:
            try:
                distances, segment_name, folder_path = load_gripper_distances(gripper_path)
                stats = compute_gripper_stats(distances)
                all_gripper_data.append((distances, segment_name, stats))
            except Exception as e:
                print(f"  Failed to load {gripper_path}: {e}")

        if all_gripper_data:
            gripper_grid_path = input_folder / 'gripper_distances_overview.jpg'
            print(f"\nGenerating gripper distance grid: {gripper_grid_path}")
            create_gripper_grid(all_gripper_data, str(gripper_grid_path))
            print(f"Gripper distance grid saved!")
    else:
        print("No gripper_distances.txt files found")

    # Summary
    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    print(f"Total trajectories: {len(trajectory_files)}")
    print(f"Processed: {processed}")
    print(f"Skipped: {skipped}")
    print(f"Failed: {failed}")
    print(f"Combined grid: {input_folder / 'trajectory_overview.jpg'}")
    if gripper_files:
        print(f"Gripper grid: {input_folder / 'gripper_distances_overview.jpg'}")
    print("="*50)


if __name__ == "__main__":
    main()
