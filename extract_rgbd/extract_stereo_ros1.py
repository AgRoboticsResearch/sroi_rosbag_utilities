import rosbag
import numpy as np
import msgReaders
import cv2
import os
import argparse

def main(bag_path, save_folder, camera_type, compressed, depth):
    bag = rosbag.Bag(bag_path)
    msgReaders.print_bag_topics(bag)
    save_folder = save_folder + os.path.basename(bag_path).replace(".bag", "") + "/"

    if camera_type == "oak":
        prefix = "/oak"
        depth_cam_info_topic = prefix + "/stereo/camera_info"
        depth_topic = "/oak/stereo/image_raw"
        right_cam_topic = prefix + "/right/image_rect_color" + ("/compressed" if compressed else "")
        left_cam_topic = prefix + "/left/image_rect_color" + ("/compressed" if compressed else "")
        right_camera_info_topic = prefix + "/right/camera_info"
        left_camera_info_topic = prefix + "/left/camera_info"

    elif camera_type == "realsense_d435i":
        prefix = "/camera"
        depth_cam_info_topic = prefix + "/aligned_depth_to_color/camera_info"
        depth_topic = "/aligned_depth_to_color/image_raw"
        right_cam_topic = prefix + "/infra2/image_rect_raw" + ("/compressed" if compressed else "")
        left_cam_topic = prefix + "/infra1/image_rect_raw" + ("/compressed" if compressed else "")
        right_camera_info_topic = prefix + "/infra2/camera_info"
        left_camera_info_topic = prefix + "/infra1/camera_info"
        color_topic = "/camera/color/image_raw" + ("/compressed" if compressed else "")
        color_camera_info_topic = "/camera/color/camera_info"

    else:
        print("Invalid camera type, need to be oak or realsense_d435i")
        return

    right_cam_timelist, right_cam_timelist_ros = msgReaders.create_timelist(bag, right_cam_topic)
    right_cam_cam_info = msgReaders.camera_info_msg(bag, right_camera_info_topic)
    proj_mat_right_cam = np.asarray(right_cam_cam_info['P']).reshape([3, 4])


    left_cam_timelist, left_cam_timelist_ros = msgReaders.create_timelist(bag, left_cam_topic)
    left_cam_cam_info = msgReaders.camera_info_msg(bag, left_camera_info_topic)
    proj_mat_left_cam = np.asarray(left_cam_cam_info['P']).reshape([3, 4])

    if camera_type == "realsense_d435i":
        color_timelist, color_timelist_ros = msgReaders.create_timelist(bag, color_topic)
        color_cam_info = msgReaders.camera_info_msg(bag, color_camera_info_topic)
        proj_mat_color = np.asarray(color_cam_info['P']).reshape([3, 4])


    # Sync timelist
    if camera_type == "realsense_d435i":
        left_cam_timelist_synced, right_cam_timelist_synced, color_timelist_synced = msgReaders.sync_msgs([left_cam_timelist, right_cam_timelist, color_timelist])
        right_cam_timelist_ros_synced = [right_cam_timelist_ros[i] for i in (right_cam_timelist_synced[:, 1].astype(dtype=int))]
        left_cam_timelist_ros_synced = [left_cam_timelist_ros[i] for i in (left_cam_timelist_synced[:, 1].astype(dtype=int))]
        color_timelist_ros_synced = [color_timelist_ros[i] for i in (color_timelist_synced[:, 1].astype(dtype=int))]

        right_cam_timelist_synced = [right_cam_timelist[i] for i in (right_cam_timelist_synced[:, 1].astype(dtype=int))]
        left_cam_timelist_synced = [left_cam_timelist[i] for i in (left_cam_timelist_synced[:, 1].astype(dtype=int))]
        color_timelist_synced = [color_timelist[i] for i in (color_timelist_synced[:, 1].astype(dtype=int))]

        print("color_timelist_ros_synced: ", len(color_timelist_ros_synced))
    
    else:

        left_cam_timelist_synced, right_cam_timelist_synced = msgReaders.sync_msgs([left_cam_timelist, right_cam_timelist])
        right_cam_timelist_ros_synced = [right_cam_timelist_ros[i] for i in (right_cam_timelist_synced[:, 1].astype(dtype=int))]
        left_cam_timelist_ros_synced = [left_cam_timelist_ros[i] for i in (left_cam_timelist_synced[:, 1].astype(dtype=int))]

        right_cam_timelist_synced = [right_cam_timelist[i] for i in (right_cam_timelist_synced[:, 1].astype(dtype=int))]
        left_cam_timelist_synced = [left_cam_timelist[i] for i in (left_cam_timelist_synced[:, 1].astype(dtype=int))]

    print("right_cam_timelist_ros_synced: ", len(right_cam_timelist_ros_synced))
    print("left_cam_timelist_ros_synced: ", len(left_cam_timelist_ros_synced))
    # check if save folder exsit and create if not
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    # save camera info in txt
    np.savetxt(save_folder + "camera_info_right.txt", proj_mat_right_cam)
    np.savetxt(save_folder + "camera_info_left.txt", proj_mat_left_cam)
    if camera_type == "realsense_d435i":
        np.savetxt(save_folder + "camera_info_color.txt", proj_mat_color)


    # open a file to save the timestamps
    with open(save_folder + "times.txt", "w") as f:
        for idx in range(len(left_cam_timelist_ros_synced)):
            f.write(str(left_cam_timelist_synced[idx][0]) + "\n")

            right_cam_image = msgReaders.retrive_image(idx, bag, right_cam_timelist_ros_synced, right_cam_topic, compressed=True)
            # right_cam_image = cv2.cvtColor(right_cam_image, cv2.COLOR_BGR2RGB)

            left_cam_image = msgReaders.retrive_image(idx, bag, left_cam_timelist_ros_synced, left_cam_topic, compressed=True)
            # left_cam_image = cv2.cvtColor(left_cam_image, cv2.COLOR_BGR2RGB)

            # save 
            cv2.imwrite(save_folder + "right_%06i.png" % idx, right_cam_image)
            cv2.imwrite(save_folder + "left_%06i.png" % idx, left_cam_image)

            if camera_type == "realsense_d435i":
                color_image = msgReaders.retrive_image(idx, bag, color_timelist_ros_synced, color_topic, compressed=True)
                # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                cv2.imwrite(save_folder + "color_%06i.png" % idx, color_image)

def parse_args():
    parser = argparse.ArgumentParser(description="Process your script's arguments")
    parser.add_argument("bag_path", help="Path to the bag file")
    parser.add_argument("save_folder", help="Path to the save folder")
    parser.add_argument("camera_type", help="oak or realsense_d435i")
    parser.add_argument("--compressed", action="store_true", help="Whether the image msgs are compressed")
    parser.add_argument("--depth", action="store_true", help="Whether to extract depth images")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print(f"bag_path: {args.bag_path}")
    print(f"save_folder: {args.save_folder}")
    print(f"camera_type: {args.camera_type}")
    print(f"compressed: {args.compressed}")
    print(f"depth: {args.depth}")
    main(args.bag_path, args.save_folder, args.camera_type, args.compressed, args.depth)