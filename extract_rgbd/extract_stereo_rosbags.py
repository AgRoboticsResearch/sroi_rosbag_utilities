#!/usr/bin/env python3
"""
Extract stereo images from ROS1 bag files using the modern rosbags library.
This script replaces the rosbag dependency with rosbags library for better performance and Python 3 compatibility.
"""

import numpy as np
import cv2
import os
import argparse
from pathlib import Path
from typing import List, Tuple, Dict, Any
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import KDTree


class RosbagImageExtractor:
    def __init__(self, bag_path: str, save_folder: str, camera_type: str, compressed: bool = False, depth: bool = False):
        self.bag_path = bag_path
        self.save_folder = save_folder + os.path.basename(bag_path).replace(".bag", "") + "/"
        self.camera_type = camera_type
        self.compressed = compressed
        self.depth = depth
        
        # Get typestore for deserialization - use ROS1 typestore for ROS1 bags
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        
        # Initialize topic mappings based on camera type
        self._setup_topic_mappings()
        
        # Ensure save folder exists
        os.makedirs(self.save_folder, exist_ok=True)
    
    def _setup_topic_mappings(self):
        """Setup topic mappings based on camera type"""
        if self.camera_type == "oak":
            prefix = "/oak"
            self.depth_cam_info_topic = prefix + "/stereo/camera_info"
            self.depth_topic = "/oak/stereo/image_raw"
            self.right_cam_topic = prefix + "/right/image_rect_color" + ("/compressed" if self.compressed else "")
            self.left_cam_topic = prefix + "/left/image_rect_color" + ("/compressed" if self.compressed else "")
            self.right_camera_info_topic = prefix + "/right/camera_info"
            self.left_camera_info_topic = prefix + "/left/camera_info"
            
        elif self.camera_type == "realsense_d435i":
            prefix = "/camera"
            self.depth_cam_info_topic = prefix + "/aligned_depth_to_color/camera_info"
            self.depth_topic = "/aligned_depth_to_color/image_raw"
            self.right_cam_topic = prefix + "/infra2/image_rect_raw" + ("/compressed" if self.compressed else "")
            self.left_cam_topic = prefix + "/infra1/image_rect_raw" + ("/compressed" if self.compressed else "")
            self.right_camera_info_topic = prefix + "/infra2/camera_info"
            self.left_camera_info_topic = prefix + "/infra1/camera_info"
            self.color_topic = prefix + "/color/image_raw" + ("/compressed" if self.compressed else "")
            self.color_camera_info_topic = prefix + "/color/camera_info"
        else:
            raise ValueError("Invalid camera type, need to be oak or realsense_d435i")
    
    def print_bag_topics(self, reader: Reader):
        """Print all topics in the bag"""
        topics = list(reader.topics.keys())
        print("Available topics:")
        for topic in topics:
            topic_info = reader.topics[topic]
            print(f"  {topic}: {topic_info.msgtype} ({topic_info.msgcount} messages)")
    
    def get_camera_info(self, reader: Reader, topic: str) -> Dict[str, Any]:
        """Extract camera info from the first message of a camera_info topic"""
        print(f"Reading camera info from topic: {topic}")
        
        # Get the connection for the topic
        connections = [x for x in reader.connections if x.topic == topic]
        if not connections:
            raise ValueError(f"Topic {topic} not found in bag file")
        
        for connection, timestamp, rawdata in reader.messages(connections):
            # Use the typestore to deserialize ROS1 messages
            msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
            
            camera_info = {
                "height": msg.height,
                "width": msg.width,
                "distortion_model": msg.distortion_model,
                "D": list(msg.D),
                "K": list(msg.K),
                "R": list(msg.R),
                "P": list(msg.P)
            }
            return camera_info
        
        raise ValueError(f"No messages found in topic: {topic}")
    
    def create_timelist(self, reader: Reader, topic: str) -> Tuple[List, List]:
        """Create timestamp lists for a topic"""
        time_list = []
        timestamps = []
        
        # Get the connection for the topic
        connections = [x for x in reader.connections if x.topic == topic]
        if not connections:
            raise ValueError(f"Topic {topic} not found in bag file")
        
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections)):
            time_sec = timestamp / 1e9  # Convert nanoseconds to seconds
            time_list.append([time_sec, i])
            timestamps.append(timestamp)
        
        print(f"{topic}: {len(time_list)} messages")
        return np.array(time_list), timestamps
    
    def sync_messages(self, msg_lists: List[np.ndarray], dt_threshold: float = None) -> List[np.ndarray]:
        """Synchronize multiple message lists based on timestamps"""
        print(f"Synchronizing {len(msg_lists)} message streams")
        
        if dt_threshold is None:
            # Calculate average period from first message stream
            msg_t = msg_lists[0][:, 0]
            dt_threshold = (msg_t[-1] - msg_t[0]) / len(msg_t)
        
        print(f"dt threshold: {dt_threshold}")
        
        msg1_t = msg_lists[0][:, 0]
        
        # Create KDTrees for other message streams
        timestamps_kd_list = []
        for msg_list in msg_lists[1:]:
            timestamps_kd = KDTree(msg_list[:, 0].reshape(-1, 1))
            timestamps_kd_list.append(timestamps_kd)
        
        # Find synchronized indices
        synced_indices = []
        for msg1_idx in range(len(msg1_t)):
            indices = [msg1_idx]
            is_valid = True
            
            for timestamps_kd in timestamps_kd_list:
                dt, msg_idx = timestamps_kd.query([[msg1_t[msg1_idx]]])
                if abs(dt[0]) > dt_threshold:
                    is_valid = False
                    break
                indices.append(msg_idx[0])
            
            if is_valid:
                synced_indices.append(indices)
        
        synced_indices = np.array(synced_indices).T
        print(f"Synchronized indices shape: {synced_indices.shape}")
        
        # Extract synchronized messages
        synced_msgs = []
        for i, msg_list in enumerate(msg_lists):
            synced_msg = msg_list[synced_indices[i]]
            synced_msgs.append(synced_msg)
        
        return synced_msgs
    
    def extract_image_from_msg(self, rawdata: bytes, connection, compressed: bool = False) -> np.ndarray:
        """Extract OpenCV image from ROS message data"""
        # Use the typestore to deserialize ROS1 messages
        msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
        
        if compressed:
            # Handle compressed image messages
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
        else:
            # Handle uncompressed image messages
            if msg.encoding == "mono8":
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_data.reshape((msg.height, msg.width))
            elif msg.encoding == "bgr8":
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_data.reshape((msg.height, msg.width, 3))
            elif msg.encoding == "rgb8":
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_data.reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "16UC1":
                img_data = np.frombuffer(msg.data, dtype=np.uint16)
                img = img_data.reshape((msg.height, msg.width))
            else:
                raise ValueError(f"Unsupported image encoding: {msg.encoding}")
        
        return img
    
    def retrieve_image_by_index(self, reader: Reader, topic: str, index: int, timestamps: List) -> np.ndarray:
        """Retrieve image by index from topic"""
        target_timestamp = timestamps[index]
        
        # Get the connection for the topic
        connections = [x for x in reader.connections if x.topic == topic]
        if not connections:
            raise ValueError(f"Topic {topic} not found in bag file")
        
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections)):
            if i == index:
                return self.extract_image_from_msg(rawdata, connection, self.compressed)
        
        raise ValueError(f"Could not find message at index {index} in topic {topic}")
    
    def extract_images(self):
        """Main extraction method"""
        print(f"Processing bag: {self.bag_path}")
        
        with Reader(self.bag_path) as reader:
            # Print available topics
            self.print_bag_topics(reader)
            
            # Get camera info
            print("Extracting camera information...")
            right_cam_info = self.get_camera_info(reader, self.right_camera_info_topic)
            left_cam_info = self.get_camera_info(reader, self.left_camera_info_topic)
            
            proj_mat_right = np.array(right_cam_info['P']).reshape(3, 4)
            proj_mat_left = np.array(left_cam_info['P']).reshape(3, 4)
            
            # Save camera info
            np.savetxt(self.save_folder + "camera_info_right.txt", proj_mat_right)
            np.savetxt(self.save_folder + "camera_info_left.txt", proj_mat_left)
            
            if self.camera_type == "realsense_d435i":
                color_cam_info = self.get_camera_info(reader, self.color_camera_info_topic)
                proj_mat_color = np.array(color_cam_info['P']).reshape(3, 4)
                np.savetxt(self.save_folder + "camera_info_color.txt", proj_mat_color)
            
            # Create timelists
            print("Creating timestamp lists...")
            right_timelist, right_timestamps = self.create_timelist(reader, self.right_cam_topic)
            left_timelist, left_timestamps = self.create_timelist(reader, self.left_cam_topic)
            
            if self.camera_type == "realsense_d435i":
                color_timelist, color_timestamps = self.create_timelist(reader, self.color_topic)
                # Synchronize all three streams
                synced_lists = self.sync_messages([left_timelist, right_timelist, color_timelist])
                left_synced, right_synced, color_synced = synced_lists
                
                print(f"Synchronized {len(left_synced)} frames")
            else:
                # Synchronize stereo streams only
                synced_lists = self.sync_messages([left_timelist, right_timelist])
                left_synced, right_synced = synced_lists
                
                print(f"Synchronized {len(left_synced)} frames")
            
            # Extract and save images
            print("Extracting and saving images...")
            with open(self.save_folder + "times.txt", "w") as f:
                for idx in range(len(left_synced)):
                    timestamp = left_synced[idx][0]
                    f.write(f"{timestamp}\n")
                    
                    # Get original indices
                    left_idx = int(left_synced[idx][1])
                    right_idx = int(right_synced[idx][1])
                    
                    # Extract images
                    left_img = self.retrieve_image_by_index(reader, self.left_cam_topic, left_idx, left_timestamps)
                    right_img = self.retrieve_image_by_index(reader, self.right_cam_topic, right_idx, right_timestamps)
                    
                    # Save images
                    cv2.imwrite(f"{self.save_folder}left_{idx:06d}.png", left_img)
                    cv2.imwrite(f"{self.save_folder}right_{idx:06d}.png", right_img)
                    
                    if self.camera_type == "realsense_d435i":
                        color_idx = int(color_synced[idx][1])
                        color_img = self.retrieve_image_by_index(reader, self.color_topic, color_idx, color_timestamps)
                        cv2.imwrite(f"{self.save_folder}color_{idx:06d}.png", color_img)
                    
                    if idx % 10 == 0:
                        print(f"Processed {idx + 1}/{len(left_synced)} frames")
        
        print(f"Extraction complete! Images saved to: {self.save_folder}")


def parse_args():
    parser = argparse.ArgumentParser(description="Extract stereo images from ROS1 bag files using rosbags library")
    parser.add_argument("bag_path", help="Path to the bag file")
    parser.add_argument("save_folder", help="Path to the save folder")
    parser.add_argument("camera_type", choices=["oak", "realsense_d435i"], help="Camera type")
    parser.add_argument("--compressed", action="store_true", help="Whether the image messages are compressed")
    parser.add_argument("--depth", action="store_true", help="Whether to extract depth images (not implemented yet)")
    return parser.parse_args()


def main():
    args = parse_args()
    
    print(f"Bag path: {args.bag_path}")
    print(f"Save folder: {args.save_folder}")
    print(f"Camera type: {args.camera_type}")
    print(f"Compressed: {args.compressed}")
    print(f"Depth: {args.depth}")
    
    # Validate inputs
    if not os.path.exists(args.bag_path):
        raise FileNotFoundError(f"Bag file not found: {args.bag_path}")
    
    # Create extractor and run
    extractor = RosbagImageExtractor(
        bag_path=args.bag_path,
        save_folder=args.save_folder,
        camera_type=args.camera_type,
        compressed=args.compressed,
        depth=args.depth
    )
    
    extractor.extract_images()


if __name__ == "__main__":
    main()
