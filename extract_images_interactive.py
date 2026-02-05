#!/usr/bin/env python3
"""
Interactive ROS1 Image Extractor

Extract images from ROS1 rosbag files using the rosbags library.
Allows interactive selection of image topics one at a time with optional MP4 encoding.
"""

import argparse
import os
import re
from typing import List, Tuple, Dict, Any

import cv2
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


class InteractiveImageExtractor:
    """Interactive extractor for image topics from ROS1 bag files."""

    def __init__(self, bag_path: str, output_dir: str = "./output"):
        """
        Initialize the extractor.

        Args:
            bag_path: Path to the ROS1 bag file
            output_dir: Base directory for extracted images
        """
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        self.bag_name = os.path.basename(bag_path).replace(".bag", "")

    def get_image_topics(self) -> List[Dict[str, Any]]:
        """
        Return list of image topics with metadata.

        Returns:
            List of dicts with keys: topic, msgtype, count, compressed
        """
        topics = []

        # Handle both naming conventions: sensor_msgs/Image and sensor_msgs/msg/Image
        image_types = [
            "sensor_msgs/Image",
            "sensor_msgs/msg/Image",
            "sensor_msgs/CompressedImage",
            "sensor_msgs/msg/CompressedImage",
        ]

        with Reader(self.bag_path) as reader:
            for topic, topic_info in reader.topics.items():
                msgtype = topic_info.msgtype
                count = topic_info.msgcount

                # Filter for image message types
                if msgtype in image_types:
                    topics.append({
                        "topic": topic,
                        "msgtype": msgtype,
                        "count": count,
                        "compressed": "CompressedImage" in msgtype
                    })

        return topics

    def display_topics(self, topics: List[Dict]):
        """
        Display topics in a numbered menu format.

        Args:
            topics: List of topic metadata dicts
        """
        print(f"\n{'='*60}")
        print(f"Bag: {self.bag_name}")
        print(f"Found {len(topics)} image topic(s):")
        print(f"{'='*60}")

        for i, topic_info in enumerate(topics):
            compressed_flag = " [compressed]" if topic_info["compressed"] else ""
            print(f"  [{i}] {topic_info['topic']}{compressed_flag}")
            print(f"      Type: {topic_info['msgtype']}, Messages: {topic_info['count']}")

        print(f"{'='*60}\n")

    def sanitize_topic_name(self, topic: str) -> str:
        """
        Convert topic name to a valid directory name.

        Args:
            topic: Original topic name

        Returns:
            Sanitized topic name safe for use as directory name
        """
        # Remove leading slashes and replace remaining slashes with underscores
        sanitized = topic.strip("/")
        sanitized = sanitized.replace("/", "_")
        # Remove any other non-alphanumeric characters (except underscores)
        sanitized = re.sub(r"[^a-zA-Z0-9_]", "_", sanitized)
        return sanitized

    def extract_images(self, topic: str) -> Tuple[int, float]:
        """
        Extract images from topic.

        Args:
            topic: Full topic name to extract

        Returns:
            Tuple of (frame_count, average_fps)
        """
        # Create output folder
        sanitized_topic = self.sanitize_topic_name(topic)
        output_folder = os.path.join(self.output_dir, self.bag_name, sanitized_topic)
        os.makedirs(output_folder, exist_ok=True)

        frame_count = 0
        timestamps = []

        print(f"\nExtracting images from: {topic}")
        print(f"Output folder: {output_folder}")

        with Reader(self.bag_path) as reader:
            # Get connections for the topic
            connections = [x for x in reader.connections if x.topic == topic]
            if not connections:
                raise ValueError(f"Topic {topic} not found in bag file")

            connection = connections[0]
            is_compressed = "CompressedImage" in connection.msgtype

            for conn, timestamp, rawdata in reader.messages(connections):
                # Extract timestamp
                time_sec = timestamp / 1e9  # Convert nanoseconds to seconds
                timestamps.append(time_sec)

                # Deserialize and decode image
                msg = self.typestore.deserialize_ros1(rawdata, conn.msgtype)
                img = self._decode_image(msg, is_compressed)

                # Save frame
                frame_path = os.path.join(output_folder, f"frame_{frame_count:06d}.jpg")
                cv2.imwrite(frame_path, img)

                frame_count += 1

                # Show progress every 50 frames
                if frame_count % 50 == 0:
                    print(f"  Extracted {frame_count} frames...")

        print(f"  Complete! Extracted {frame_count} frames.")

        # Calculate average FPS
        if len(timestamps) > 1:
            total_duration = timestamps[-1] - timestamps[0]
            avg_fps = (frame_count - 1) / total_duration if total_duration > 0 else 30.0
        else:
            avg_fps = 30.0

        return frame_count, avg_fps

    def _decode_image(self, msg, is_compressed: bool) -> np.ndarray:
        """
        Decode ROS image message to OpenCV image.

        Args:
            msg: ROS message
            is_compressed: Whether the message is compressed

        Returns:
            OpenCV image as numpy array
        """
        if is_compressed:
            # Handle compressed image messages
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
            return img
        else:
            # Handle uncompressed image messages
            if msg.encoding == "mono8":
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_data.reshape((msg.height, msg.width))
                # Convert to BGR for consistent saving
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif msg.encoding == "bgr8":
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_data.reshape((msg.height, msg.width, 3))
            elif msg.encoding == "rgb8":
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_data.reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "16UC1":
                # Depth image - normalize to 8-bit for visualization
                img_data = np.frombuffer(msg.data, dtype=np.uint16)
                depth_img = img_data.reshape((msg.height, msg.width))
                # Normalize to 0-255 range
                depth_img = cv2.normalize(depth_img, np.array([]), 0, 255, cv2.NORM_MINMAX)
                img = cv2.cvtColor(depth_img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
            elif msg.encoding == "bayer_rggb8":
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_data.reshape((msg.height, msg.width))
                img = cv2.cvtColor(img, cv2.COLOR_BayerBG2BGR)
            else:
                raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        return img

    def encode_mp4(self, topic: str, fps: float) -> str:
        """
        Encode extracted images to MP4.

        Args:
            topic: Topic name (used to find the image folder)
            fps: Frame rate for the output video

        Returns:
            Path to the output MP4 file
        """
        sanitized_topic = self.sanitize_topic_name(topic)
        input_folder = os.path.join(self.output_dir, self.bag_name, sanitized_topic)
        output_path = os.path.join(input_folder, "video.mp4")

        # Find all frame files
        frame_files = sorted([
            f for f in os.listdir(input_folder)
            if f.startswith("frame_") and f.endswith(".jpg")
        ])

        if not frame_files:
            raise ValueError(f"No frame files found in {input_folder}")

        print(f"\nEncoding MP4 from {len(frame_files)} frames at {fps:.2f} FPS...")

        # Read first frame to get dimensions
        first_frame = cv2.imread(os.path.join(input_folder, frame_files[0]))
        height, width = first_frame.shape[:2]

        # Create video writer
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

        for i, frame_file in enumerate(frame_files):
            frame_path = os.path.join(input_folder, frame_file)
            frame = cv2.imread(frame_path)
            writer.write(frame)

            if len(frame_files) > 100 and i % 100 == 0:
                print(f"  Processed {i} frames...")

        writer.release()
        print(f"  MP4 saved to: {output_path}")

        return output_path


def get_user_choice(topics: List[Dict]) -> str | None:
    """
    Prompt user to select a topic by number.

    Args:
        topics: List of topic metadata dicts

    Returns:
        Selected topic name, or None if user wants to quit
    """
    while True:
        try:
            user_input = input("Enter topic number to extract (or 'q' to quit): ").strip()

            if user_input.lower() == "q":
                return None

            index = int(user_input)
            if 0 <= index < len(topics):
                return topics[index]["topic"]
            else:
                print(f"Invalid index. Please enter a number between 0 and {len(topics) - 1}.")
        except ValueError:
            print("Invalid input. Please enter a number or 'q'.")


def ask_yes_no(prompt: str) -> bool:
    """
    Ask user a yes/no question.

    Args:
        prompt: Question to ask

    Returns:
        True if user answers yes, False otherwise
    """
    while True:
        response = input(prompt).strip().lower()
        if response in ["y", "yes"]:
            return True
        elif response in ["n", "no"]:
            return False
        else:
            print("Please enter 'y' or 'n'.")


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Interactively extract images from ROS1 bag files"
    )
    parser.add_argument(
        "bag_path",
        nargs="?",
        help="Path to the ROS1 bag file"
    )
    parser.add_argument(
        "--output", "-o",
        default="./output",
        help="Output directory for extracted images (default: ./output)"
    )
    return parser.parse_args()


def main():
    """Main entry point for the interactive extractor."""
    args = parse_args()

    # Get bag path if not provided
    bag_path = args.bag_path
    if not bag_path:
        bag_path = input("Enter path to ROS1 bag file: ").strip()

    # Validate bag file exists
    if not os.path.exists(bag_path):
        print(f"Error: Bag file not found: {bag_path}")
        return 1

    # Create extractor
    extractor = InteractiveImageExtractor(bag_path, args.output)

    # Get available image topics
    topics = extractor.get_image_topics()

    if not topics:
        print("No image topics found in the bag file.")
        return 0

    # Main interactive loop
    while True:
        # Display available topics
        extractor.display_topics(topics)

        # Get user selection
        selected_topic = get_user_choice(topics)

        if selected_topic is None:
            print("Exiting.")
            break

        # Ask about MP4 encoding BEFORE extracting
        encode_mp4 = ask_yes_no("Encode to MP4? (y/n): ")

        # Extract images
        try:
            frame_count, avg_fps = extractor.extract_images(selected_topic)
            print(f"Detected average FPS: {avg_fps:.2f}")
        except Exception as e:
            print(f"Error extracting images: {e}")
            continue

        # Encode to MP4 if requested
        if encode_mp4:
            try:
                extractor.encode_mp4(selected_topic, avg_fps)
            except Exception as e:
                print(f"Error encoding MP4: {e}")

        # Ask if user wants to extract another topic
        if not ask_yes_no("Extract another topic? (y/n): "):
            print("Exiting.")
            break

    return 0


if __name__ == "__main__":
    exit(main())
