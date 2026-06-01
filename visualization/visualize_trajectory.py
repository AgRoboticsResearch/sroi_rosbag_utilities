#!/usr/bin/env python3
"""
Interactive 3D Trajectory Visualization using Plotly.

Usage:
    python visualize_trajectory.py ./recordings/1777418620/episode_001/CameraTrajectory.txt
    python visualize_trajectory.py ./recordings/1777418620/episode_001/CameraTrajectoryTransformed.txt
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import plotly.graph_objects as go


def load_kitti_trajectory(path):
    """Load KITTI-format trajectory (12 or 13 floats per line)."""
    traj = np.loadtxt(path)
    # Handle 1D single-line case
    if traj.ndim == 1:
        traj = traj.reshape(1, -1)
    # Drop timestamp column if present (13 values -> 12)
    if traj.ndim == 2 and traj.shape[1] == 13:
        traj = traj[:, 1:]
    traj = traj.reshape(-1, 3, 4)
    # Append [0 0 0 1] to make 4x4
    append = np.zeros((traj.shape[0], 1, 4))
    append[:, 0, 3] = 1
    traj = np.concatenate([traj, append], axis=1)
    return traj


def load_sroi_json(path):
    """Load SROI JSON trajectory (pos + quaternion per frame)."""
    with open(path) as f:
        data = json.load(f)
    frames = data["frames"]
    positions = np.array([fr["pos"] for fr in frames])
    rotations = np.array([fr["rot"] for fr in frames])  # [x,y,z,w] quaternions
    timestamps = np.array([fr["t"] for fr in frames])
    # Build 4x4 transforms
    from scipy.spatial.transform import Rotation
    n = len(frames)
    traj = np.eye(4)[np.newaxis].repeat(n, axis=0)
    traj[:, :3, 3] = positions
    traj[:, :3, :3] = Rotation.from_quat(rotations).as_matrix()
    return traj


def visualize(trajectory_path):
    path = Path(trajectory_path)
    if path.suffix == ".json":
        traj = load_sroi_json(trajectory_path)
    else:
        traj = load_kitti_trajectory(trajectory_path)
    x, y, z = traj[:, 0, 3], traj[:, 1, 3], traj[:, 2, 3]

    # Compute path length
    diffs = np.diff(np.column_stack([x, y, z]), axis=0)
    total_length = np.sum(np.linalg.norm(diffs, axis=1))

    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=x,
                y=y,
                z=z,
                mode="lines+markers",
                marker=dict(
                    size=3,
                    color=list(range(len(x))),
                    colorscale="Viridis",
                    colorbar=dict(title="Frame"),
                ),
                line=dict(width=3, color="blue"),
                name="Trajectory",
            )
        ]
    )

    # Mark start and end
    fig.add_trace(
        go.Scatter3d(
            x=[x[0]], y=[y[0]], z=[z[0]],
            mode="markers", marker=dict(size=8, color="green"),
            name="Start",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=[x[-1]], y=[y[-1]], z=[z[-1]],
            mode="markers", marker=dict(size=8, color="red"),
            name="End",
        )
    )

    title = f"Trajectory: {Path(trajectory_path).name}"
    title += f"<br>{len(traj)} frames, {total_length:.3f}m path length"

    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title="X",
            yaxis_title="Y",
            zaxis_title="Z",
            aspectmode="data",
        ),
        width=900,
        height=700,
    )

    fig.show()

    # Save plot as JPG using matplotlib (avoids kaleido version issues)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig_mpl, ax = plt.subplots(figsize=(8, 8), subplot_kw={'projection': '3d'})
    ax.plot(x, y, z, '-b', linewidth=1.5)
    ax.scatter([x[0]], [y[0]], [z[0]], c='green', s=60, label='Start')
    ax.scatter([x[-1]], [y[-1]], [z[-1]], c='red', s=60, label='End')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title(f"{path.name}\n{len(traj)} frames, {total_length:.3f}m")
    ax.legend()
    out_path = str(path.with_suffix(".jpg"))
    fig_mpl.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig_mpl)
    print(f"Saved plot to {out_path}")

    print(f"Path length: {total_length:.3f}m")
    print(f"Frames: {len(traj)}")
    print(f"X range: [{x.min():.4f}, {x.max():.4f}]")
    print(f"Y range: [{y.min():.4f}, {y.max():.4f}]")
    print(f"Z range: [{z.min():.4f}, {z.max():.4f}]")


def main():
    parser = argparse.ArgumentParser(description="Interactive 3D trajectory visualization")
    parser.add_argument("trajectory", help="Path to CameraTrajectory.txt or CameraTrajectoryTransformed.txt")
    args = parser.parse_args()

    if not Path(args.trajectory).exists():
        print(f"File not found: {args.trajectory}")
        sys.exit(1)

    visualize(args.trajectory)


if __name__ == "__main__":
    main()
