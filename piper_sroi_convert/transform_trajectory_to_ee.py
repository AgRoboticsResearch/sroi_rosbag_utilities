import argparse
import os
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def load_slam_trajectory(path):
    """Load a CameraTrajectory.txt (12-column 3x4 matrices) into 4x4 poses."""
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


def save_slam_trajectory(path, poses):
    """Save poses as a CameraTrajectory.txt-compatible file (12 columns per line)."""
    with open(path, 'w') as f:
        for T in poses:
            vals = [
                T[0, 0], T[0, 1], T[0, 2], T[0, 3],
                T[1, 0], T[1, 1], T[1, 2], T[1, 3],
                T[2, 0], T[2, 1], T[2, 2], T[2, 3],
            ]
            line = " ".join(f"{v:.9f}" for v in vals)
            f.write(line + "\n")


def get_T_ee_cam():
    """Return the fixed camera-in-EE transform derived from URDF extrinsics."""
    # camera_link in link6 frame
    T_link6_cam = np.eye(4)
    T_link6_cam[:3, :3] = Rotation.from_euler('xyz', [-0.640492, 0, -1.570797]).as_matrix()
    T_link6_cam[:3, 3] = [-0.054333, 0.00895, 0.013471]

    # ee_link in link6 frame
    T_link6_ee = np.eye(4)
    T_link6_ee[:3, :3] = Rotation.from_euler('xyz', [0, 0, -1.5707963]).as_matrix()
    T_link6_ee[:3, 3] = [0, 0, 0.116517]

    T_ee_cam = np.linalg.inv(T_link6_ee) @ T_link6_cam
    return T_ee_cam


def process_single(input_path, output_path):
    """Transform one CameraTrajectory.txt to EE frame.

    Each pose T_w_cam represents the camera pose in world frame (maps cam -> world).
    The EE pose is obtained by composing with the fixed extrinsic T_cam_ee:
        T_w_ee = T_w_cam @ T_cam_ee
    """
    T_ee_cam = get_T_ee_cam()
    T_cam_ee = np.linalg.inv(T_ee_cam)

    print(f"T_ee_cam:\n{np.round(T_ee_cam, 4)}")
    print(f"T_cam_ee:\n{np.round(T_cam_ee, 4)}")

    poses_cam = load_slam_trajectory(input_path)
    print(f"Loaded {len(poses_cam)} poses from {input_path}")

    poses_ee = []
    for T_w_cam in poses_cam:
        T_w_ee = T_w_cam @ T_cam_ee
        poses_ee.append(T_w_ee)

    save_slam_trajectory(output_path, poses_ee)
    print(f"Saved {len(poses_ee)} EE poses to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Transform CameraTrajectory.txt (camera poses) to EETrajectory.txt (end-effector poses)"
    )
    parser.add_argument("--input", help="Input CameraTrajectory.txt (single-file mode)")
    parser.add_argument("--output", help="Output EETrajectory.txt (single-file mode)")
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search INPUT directory recursively for CameraTrajectory.txt and convert each",
    )
    args = parser.parse_args()

    if args.recursive:
        if not args.input:
            parser.error("--input is required with --recursive (top-level directory)")
        root = Path(args.input)
        if not root.is_dir():
            parser.error(f"{root} is not a directory")
        processed = 0
        for dirpath, _, filenames in os.walk(root):
            if "CameraTrajectory.txt" not in filenames:
                continue
            ep_dir = Path(dirpath)
            cam_path = ep_dir / "CameraTrajectory.txt"
            ee_path = ep_dir / "EETrajectory.txt"
            process_single(str(cam_path), str(ee_path))
            processed += 1
        print(f"\nDone. Processed {processed} episode(s).")
    else:
        if not args.input or not args.output:
            parser.error("--input and --output are required (or use --recursive)")
        process_single(args.input, args.output)


if __name__ == "__main__":
    main()
