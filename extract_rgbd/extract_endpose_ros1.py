import rosbag
import numpy as np
import msgReaders
import cv2
import os
import argparse

def main(bag_path, save_folder, end_pose_topic):
    bag = rosbag.Bag(bag_path)
    msgReaders.print_bag_topics(bag)
    save_folder = save_folder + os.path.basename(bag_path).replace(".bag", "") + "/"
    # check if save folder exsit and create if not
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    end_pose_msgs = msgReaders.general_msg_reader(bag, end_pose_topic)
    # this is a Type: geometry_msgs/Pose


    # open a file to save the timestamps
    with open(save_folder + "OpticalEndPoseRobotArmCalc.txt", "w") as f:
        for msg in end_pose_msgs:
            # print(msg)
            timestamp = msg[0].to_sec()
            # write t, pose
            f.write(str(timestamp) + ",")
            f.write(str(msg[1].position.x) + ",")
            f.write(str(msg[1].position.y) + ",")
            f.write(str(msg[1].position.z) + ",")
            f.write(str(msg[1].orientation.x) + ",")
            f.write(str(msg[1].orientation.y) + ",")
            f.write(str(msg[1].orientation.z) + ",")
            f.write(str(msg[1].orientation.w) + "\n")

    bag.close()
    print("End Pose extraction done!")

def parse_args():
    parser = argparse.ArgumentParser(description="Process your script's arguments")
    parser.add_argument("bag_path", help="Path to the bag file")
    parser.add_argument("save_folder", help="Path to the save folder")
    parser.add_argument("end_pose_topic", help="e.g.: /camera_optical_pose_from_tf")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print(f"bag_path: {args.bag_path}")
    print(f"save_folder: {args.save_folder}")
    print(f"end_pose_topic: {args.end_pose_topic}")
    main(args.bag_path, args.save_folder, args.end_pose_topic)