#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import glob
import os
import argparse

# Print not using scientific notation
np.set_printoptions(suppress=True)

def create_transformation_matrix(xyz, quat):
    """
    Create a 4x4 transformation matrix from position (x, y, z) and quaternion (qx, qy, qz, qw).
    """
    # Convert quaternion to rotation matrix using scipy
    rotation = R.from_quat(quat)
    rotation_matrix = rotation.as_matrix()  # 3x3 rotation matrix

    # Create 4x4 transformation matrix
    transformation_matrix = np.eye(4)  # Start with an identity matrix
    transformation_matrix[:3, :3] = rotation_matrix  # Set the top-left 3x3 block to the rotation matrix
    transformation_matrix[:3, 3] = xyz  # Set the translation vector

    return transformation_matrix

def process_traj(traj_path):
    print("Processing: ", traj_path)
    # Load ORB-SLAM trajectory
    orb_trajs = np.loadtxt(traj_path +"/CameraTrajectory.txt")
    orb_time_stamps = np.loadtxt(traj_path +"/times.txt")
    orb_trajs = orb_trajs.reshape(-1, 3, 4)

    end_rows = np.zeros((orb_trajs.shape[0], 1, 4))
    end_rows[:, 0, 3] = 1
    orb_trajs = np.concatenate((orb_trajs, end_rows), axis=1)

    print("orb_trajs: ", orb_trajs.shape)
    print("orb_time_stamps: ", orb_time_stamps.shape)   

    # Load ground truth trajectory
    gt_trajs = np.loadtxt(traj_path +"/OpticalEndPoseRobotArmCalc.txt", delimiter=",")
    # gt_trajs is a list of [time, x, y, z, qx, qy, qz, qw]
    # convert to 4x4 transformation matrix
    gt_tranjs_mat = np.zeros((gt_trajs.shape[0], 4, 4))
    for i in range(gt_trajs.shape[0]):
        xyz = gt_trajs[i, 1:4]
        quat = gt_trajs[i, 4:]
        gt_tranjs_mat[i] = create_transformation_matrix(xyz, quat)
    gt_time_stamps = gt_trajs[:, 0]
    print("gt_tranjs_mat: ", gt_tranjs_mat.shape)
    print("gt_time_stamps: ", gt_time_stamps.shape)

    # Trans ground truth to the initial frame
    init_trans = gt_tranjs_mat[0]
    init_trans_inv = np.linalg.inv(init_trans)

    gt_tranjs_mat_from_inits =[]
    for i in range(gt_tranjs_mat.shape[0]):
        gt_tranjs_mat_from_init = init_trans_inv.dot(gt_tranjs_mat[i])
        gt_tranjs_mat_from_inits.append(gt_tranjs_mat_from_init)
    gt_tranjs_mat_from_inits = np.array(gt_tranjs_mat_from_inits)

    # sync two trajectories
    print("orb_time_stamps: ", orb_time_stamps.shape)
    print("gt_time_stamps: ", gt_time_stamps.shape)

    # for each orb timestamp, find the closest gt timestamp
    gt_indices = []
    for i in range(orb_time_stamps.shape[0]):
        orb_time = orb_time_stamps[i]
        gt_index = np.argmin(np.abs(gt_time_stamps - orb_time))
        gt_indices.append(gt_index)

    gt_tranjs_mat_from_inits_synced = gt_tranjs_mat_from_inits[gt_indices]
    print("gt_tranjs_mat_from_inits_synced: ", gt_tranjs_mat_from_inits_synced.shape)

    # save orb and gt trajectories to kitti format 12 numbers per line
    with open(traj_path + "/SLAM_traj.txt", "w") as f:
        for i in range(orb_trajs.shape[0]):
            for row in range(3):
                for col in range(4):
                    f.write(str(orb_trajs[i, row, col]))
                    if row < 2 or col < 3:  # Add space only if it's not the last number
                        f.write(" ")
            f.write("\n")

    with open(traj_path + "/Arm_traj.txt", "w") as f:
        for i in range(gt_tranjs_mat_from_inits_synced.shape[0]):
            for row in range(3):
                for col in range(4):
                    f.write(str(gt_tranjs_mat_from_inits_synced[i, row, col]))
                    if row < 2 or col < 3:  # Add space only if it's not the last number
                        f.write(" ")
            f.write("\n")

    # plot two subplots
    plt.figure(figsize=(15, 5))
    ax1 = plt.subplot(1, 2, 1)
    ax2 = plt.subplot(1, 2, 2)

    ax1.plot(orb_trajs[:, 0, 3], orb_trajs[:, 1, 3], label="orb")
    ax1.plot(gt_tranjs_mat_from_inits[:, 0, 3], gt_tranjs_mat_from_inits[:, 1, 3], label="robot_arm")
    ax1.legend()
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title("xy")

    ax2.plot(orb_trajs[:, 0, 3], orb_trajs[:, 2, 3], label="orb")
    ax2.plot(gt_tranjs_mat_from_inits[:, 0, 3], gt_tranjs_mat_from_inits[:, 2, 3], label="robot_arm")
    ax2.legend()
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("z (m)")
    ax2.set_title("xz")
    plt.savefig(traj_path + "/orb_gt.png")
    plt.close()  # Close the figure to free memory

def main():
    parser = argparse.ArgumentParser(description="Process trajectory data.")
    parser.add_argument("--traj_paths", type=str, required=True, help="Path pattern to trajectory directories")

    args = parser.parse_args()

    traj_paths = glob.glob(args.traj_paths)

    print(traj_paths)

    for traj_path in traj_paths:
        process_traj(traj_path)

if __name__ == "__main__":
    main()