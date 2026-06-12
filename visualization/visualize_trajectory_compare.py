#!/usr/bin/env python3
"""
Compare SLAM trajectory from camera frame vs gripper tip frame.

Visualizes both trajectories side by side in 3D Plotly views.

Usage:
    python visualize_trajectory_compare.py ./recordings/1777552826/episode_001/CameraTrajectory.txt
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

T_CAMERA_TO_EE = np.array([
    [ 1.21194748e-18,  8.01801645e-01,  5.97590262e-01,  5.44031614e-02],
    [-3.32679490e-06, -5.97590262e-01,  8.01801645e-01,  8.79707044e-02],
    [ 1.00000000e+00, -1.98806023e-06,  2.66742962e-06,  5.43332927e-02],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00],
])


def load_kitti_trajectory(path):
    traj = np.loadtxt(path)
    if traj.ndim == 1:
        traj = traj.reshape(1, -1)
    # Drop timestamp column if present (13 values -> 12)
    if traj.ndim == 2 and traj.shape[1] == 13:
        traj = traj[:, 1:]
    traj = traj.reshape(-1, 3, 4)
    append = np.zeros((traj.shape[0], 1, 4))
    append[:, 0, 3] = 1
    traj = np.concatenate([traj, append], axis=1)
    return traj


def transform_trajectory(traj_cam, T_cam_ee):
    """Apply fixed transform to convert camera trajectory to EE trajectory."""
    traj_ee = np.zeros_like(traj_cam)
    for i in range(len(traj_cam)):
        traj_ee[i] = traj_cam[i] @ T_cam_ee
    return traj_ee


def add_trajectory(fig, traj, row, col, title, color):
    x, y, z = traj[:, 0, 3], traj[:, 1, 3], traj[:, 2, 3]
    diffs = np.diff(np.column_stack([x, y, z]), axis=0)
    path_length = np.sum(np.linalg.norm(diffs, axis=1))

    fig.add_trace(
        go.Scatter3d(
            x=x, y=y, z=z,
            mode="lines+markers",
            marker=dict(size=2, color=list(range(len(x))),
                        colorscale="Viridis", colorbar=dict(title="Frame", len=0.5)),
            line=dict(width=3, color=color),
            name=title,
        ), row=row, col=col
    )
    # Start/end markers
    fig.add_trace(
        go.Scatter3d(x=[x[0]], y=[y[0]], z=[z[0]],
                     mode="markers", marker=dict(size=6, color="green"),
                     name=f"{title} Start"), row=row, col=col
    )
    fig.add_trace(
        go.Scatter3d(x=[x[-1]], y=[y[-1]], z=[z[-1]],
                     mode="markers", marker=dict(size=6, color="red"),
                     name=f"{title} End"), row=row, col=col
    )

    title += f"<br>{len(traj)} frames, {path_length:.3f}m"
    return title


def main():
    parser = argparse.ArgumentParser(description="Compare camera vs gripper tip trajectory")
    parser.add_argument("trajectory", help="Path to CameraTrajectory.txt")
    args = parser.parse_args()

    if not Path(args.trajectory).exists():
        print(f"File not found: {args.trajectory}")
        sys.exit(1)

    traj_cam = load_kitti_trajectory(args.trajectory)
    traj_ee = transform_trajectory(traj_cam, T_CAMERA_TO_EE)

    fig = make_subplots(
        rows=1, cols=2,
        specs=[[{"type": "scene"}, {"type": "scene"}]],
        subplot_titles=["Camera (SLAM output)", "Gripper Tip (transformed)"],
    )

    add_trajectory(fig, traj_cam, 1, 1, "Camera", "blue")
    add_trajectory(fig, traj_ee, 1, 2, "Gripper Tip", "orange")

    fig.update_layout(
        title=f"Trajectory Comparison: {Path(args.trajectory).parent.name}",
        width=1600, height=700,
    )
    # Same axis labels for both
    fig.update_scenes(
        xaxis_title="X", yaxis_title="Y", zaxis_title="Z",
        aspectmode="data",
    )

    fig.show()

    # Print stats
    for name, traj in [("Camera", traj_cam), ("Gripper Tip", traj_ee)]:
        x, y, z = traj[:, 0, 3], traj[:, 1, 3], traj[:, 2, 3]
        diffs = np.diff(np.column_stack([x, y, z]), axis=0)
        length = np.sum(np.linalg.norm(diffs, axis=1))
        print(f"\n{name}:")
        print(f"  Path length: {length:.4f}m")
        print(f"  X: [{x.min():.4f}, {x.max():.4f}]")
        print(f"  Y: [{y.min():.4f}, {y.max():.4f}]")
        print(f"  Z: [{z.min():.4f}, {z.max():.4f}]")


if __name__ == "__main__":
    main()
