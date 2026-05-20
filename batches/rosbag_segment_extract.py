#!/usr/bin/env python3
"""
ROS Bag Segment and Extract Pipeline.

Combines segmentation and image extraction into a single pipeline:
1. Segment ROS bags based on /upi/status/is_action topic
2. Extract synchronized stereo/RGB images from each segment

Usage:
    python rosbag_segment_extract.py -i input_dir/ -o output_dir/ realsense_d435i
    python rosbag_segment_extract.py -i input_dir/ -o output_dir/ oak --compressed
"""

import argparse
import json
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import KDTree


# =============================================================================
# Segmentation Functions (from rosbag_segment_rosbags.py)
# =============================================================================

@dataclass
class Segment:
    """Represents a continuous segment where is_action is True."""
    start_time: float  # seconds
    end_time: float    # seconds
    index: int         # segment number within the bag


def find_action_segments(bag_path: Path) -> List[Segment]:
    """Find continuous segments where /upi/status/is_action is True."""
    segments = []
    typestore = get_typestore(Stores.ROS1_NOETIC)

    with Reader(bag_path) as reader:
        status_conns = [c for c in reader.connections
                       if c.topic == '/upi/status/is_action']

        if not status_conns:
            return segments

        status_messages = []
        for conn, timestamp, rawdata in reader.messages(status_conns):
            msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
            time_sec = timestamp / 1e9
            status_messages.append((time_sec, msg.data))

        if not status_messages:
            return segments

        status_messages.sort(key=lambda x: x[0])

        segment_start = None
        segment_index = 0

        for time_sec, is_action in status_messages:
            if is_action and segment_start is None:
                segment_start = time_sec
            elif not is_action and segment_start is not None:
                segments.append(Segment(
                    start_time=segment_start,
                    end_time=time_sec,
                    index=segment_index + 1
                ))
                segment_index += 1
                segment_start = None

        if segment_start is not None:
            segments.append(Segment(
                start_time=segment_start,
                end_time=status_messages[-1][0],
                index=segment_index + 1
            ))

    return segments


def extract_segment_bag(source_path: Path, output_path: Path, segment: Segment) -> int:
    """Extract a segment from source bag to a new bag file."""
    msg_count = 0

    with Reader(source_path) as reader:
        with Writer(output_path) as writer:
            conn_map = {}
            for conn in reader.connections:
                new_conn = writer.add_connection(
                    conn.topic,
                    conn.msgtype,
                    msgdef=conn.msgdef.data,
                    md5sum=conn.digest,
                )
                conn_map[conn.id] = new_conn

            for conn, timestamp, rawdata in reader.messages():
                time_sec = timestamp / 1e9
                if segment.start_time <= time_sec <= segment.end_time:
                    writer.write(conn_map[conn.id], timestamp, rawdata)
                    msg_count += 1

    return msg_count


# =============================================================================
# Image Extraction Functions (from extract_stereo_rosbags.py)
# =============================================================================

class ImageExtractor:
    """Extract synchronized images from a ROS bag."""

    def __init__(self, camera_type: str, compressed: bool = False):
        self.camera_type = camera_type
        self.compressed = compressed
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        self._setup_topic_mappings()

    def _setup_topic_mappings(self):
        """Setup topic mappings based on camera type."""
        if self.camera_type == "oak":
            prefix = "/oak"
            self.right_cam_topic = prefix + "/right/image_rect_color" + ("/compressed" if self.compressed else "")
            self.left_cam_topic = prefix + "/left/image_rect_color" + ("/compressed" if self.compressed else "")
            self.right_camera_info_topic = prefix + "/right/camera_info"
            self.left_camera_info_topic = prefix + "/left/camera_info"
        elif self.camera_type == "realsense_d435i":
            prefix = "/camera"
            self.right_cam_topic = prefix + "/infra2/image_rect_raw" + ("/compressed" if self.compressed else "")
            self.left_cam_topic = prefix + "/infra1/image_rect_raw" + ("/compressed" if self.compressed else "")
            self.right_camera_info_topic = prefix + "/infra2/camera_info"
            self.left_camera_info_topic = prefix + "/infra1/camera_info"
            self.color_topic = prefix + "/color/image_raw" + ("/compressed" if self.compressed else "")
            self.color_camera_info_topic = prefix + "/color/camera_info"
        else:
            raise ValueError(f"Invalid camera type: {self.camera_type}")

    def get_camera_info(self, reader: Reader, topic: str) -> dict:
        """Extract camera info from the first message."""
        connections = [x for x in reader.connections if x.topic == topic]
        if not connections:
            raise ValueError(f"Topic {topic} not found")

        for connection, timestamp, rawdata in reader.messages(connections):
            msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
            return {
                "height": msg.height,
                "width": msg.width,
                "distortion_model": msg.distortion_model,
                "D": list(msg.D),
                "K": list(msg.K),
                "R": list(msg.R),
                "P": list(msg.P)
            }
        raise ValueError(f"No messages in topic: {topic}")

    def create_timelist(self, reader: Reader, topic: str) -> tuple:
        """Create timestamp lists for a topic."""
        time_list = []
        timestamps = []

        connections = [x for x in reader.connections if x.topic == topic]
        if not connections:
            raise ValueError(f"Topic {topic} not found")

        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections)):
            time_sec = timestamp / 1e9
            time_list.append([time_sec, i])
            timestamps.append(timestamp)

        return np.array(time_list), timestamps

    def sync_messages(self, msg_lists: List[np.ndarray], dt_threshold: float = None) -> List[np.ndarray]:
        """Synchronize multiple message streams based on timestamps."""
        if dt_threshold is None:
            msg_t = msg_lists[0][:, 0]
            dt_threshold = (msg_t[-1] - msg_t[0]) / len(msg_t)

        msg1_t = msg_lists[0][:, 0]
        timestamps_kd_list = [KDTree(ml[:, 0].reshape(-1, 1)) for ml in msg_lists[1:]]

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
        return [msg_list[synced_indices[i]] for i, msg_list in enumerate(msg_lists)]

    def extract_image_from_msg(self, rawdata: bytes, connection) -> np.ndarray:
        """Extract OpenCV image from ROS message data."""
        msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)

        if self.compressed:
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
        else:
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
                raise ValueError(f"Unsupported encoding: {msg.encoding}")

        return img

    def retrieve_image_by_index(self, reader: Reader, topic: str, index: int, timestamps: List) -> np.ndarray:
        """Retrieve image by index from topic."""
        connections = [x for x in reader.connections if x.topic == topic]
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections)):
            if i == index:
                return self.extract_image_from_msg(rawdata, connection)
        raise ValueError(f"Index {index} not found in {topic}")

    def extract_images(self, bag_path: Path, output_dir: Path) -> dict:
        """
        Extract synchronized images from a bag file.

        Returns:
            Dictionary with extraction stats
        """
        output_dir.mkdir(parents=True, exist_ok=True)

        with Reader(bag_path) as reader:
            # Get camera info
            right_cam_info = self.get_camera_info(reader, self.right_camera_info_topic)
            left_cam_info = self.get_camera_info(reader, self.left_camera_info_topic)

            # Save camera info
            np.savetxt(output_dir / "camera_info_right.txt", np.array(right_cam_info['P']).reshape(3, 4))
            np.savetxt(output_dir / "camera_info_left.txt", np.array(left_cam_info['P']).reshape(3, 4))

            with open(output_dir / "camera_info_right.json", "w") as f:
                json.dump(right_cam_info, f, indent=2)
            with open(output_dir / "camera_info_left.json", "w") as f:
                json.dump(left_cam_info, f, indent=2)

            if self.camera_type == "realsense_d435i":
                color_cam_info = self.get_camera_info(reader, self.color_camera_info_topic)
                np.savetxt(output_dir / "camera_info_color.txt", np.array(color_cam_info['P']).reshape(3, 4))
                with open(output_dir / "camera_info_color.json", "w") as f:
                    json.dump(color_cam_info, f, indent=2)

            # Create timelists and sync
            right_timelist, right_timestamps = self.create_timelist(reader, self.right_cam_topic)
            left_timelist, left_timestamps = self.create_timelist(reader, self.left_cam_topic)

            if self.camera_type == "realsense_d435i":
                color_timelist, color_timestamps = self.create_timelist(reader, self.color_topic)
                synced_lists = self.sync_messages([left_timelist, right_timelist, color_timelist])
                left_synced, right_synced, color_synced = synced_lists
            else:
                synced_lists = self.sync_messages([left_timelist, right_timelist])
                left_synced, right_synced = synced_lists

            frame_count = len(left_synced)

            # Extract and save images
            with open(output_dir / "times.txt", "w") as f:
                for idx in range(frame_count):
                    timestamp = left_synced[idx][0]
                    f.write(f"{timestamp}\n")

                    left_idx = int(left_synced[idx][1])
                    right_idx = int(right_synced[idx][1])

                    left_img = self.retrieve_image_by_index(reader, self.left_cam_topic, left_idx, left_timestamps)
                    right_img = self.retrieve_image_by_index(reader, self.right_cam_topic, right_idx, right_timestamps)

                    cv2.imwrite(str(output_dir / f"left_{idx:06d}.png"), left_img)
                    cv2.imwrite(str(output_dir / f"right_{idx:06d}.png"), right_img)

                    if self.camera_type == "realsense_d435i":
                        color_idx = int(color_synced[idx][1])
                        color_img = self.retrieve_image_by_index(reader, self.color_topic, color_idx, color_timestamps)
                        cv2.imwrite(str(output_dir / f"color_{idx:06d}.png"), color_img)

        return {"frame_count": frame_count}


# =============================================================================
# Combined Pipeline
# =============================================================================

def process_bag(bag_path: Path, output_dir: Path, extractor: ImageExtractor,
                min_duration: float = 5.0, skip_existing: bool = False) -> List[dict]:
    """
    Process a single bag: segment and extract images.

    Args:
        bag_path: Path to source bag
        output_dir: Output directory
        extractor: ImageExtractor instance
        min_duration: Minimum segment duration in seconds
        skip_existing: If True, skip segments that already have extracted images

    Returns:
        List of segment info dictionaries
    """
    bag_name = bag_path.stem
    results = []

    print(f"\nProcessing: {bag_path.name}")

    # Find segments
    segments = find_action_segments(bag_path)

    if not segments:
        print(f"  No action segments found")
        return results

    # Filter short segments
    if min_duration > 0:
        original_count = len(segments)
        segments = [s for s in segments if (s.end_time - s.start_time) >= min_duration]
        filtered_count = original_count - len(segments)
        if filtered_count > 0:
            print(f"  Filtered {filtered_count} segment(s) shorter than {min_duration}s")

    if not segments:
        print(f"  No segments after filtering")
        return results

    print(f"  Processing {len(segments)} segment(s)...")

    # Create directory for segment bags
    segment_bags_dir = output_dir / "segment_bags"
    segment_bags_dir.mkdir(parents=True, exist_ok=True)

    for segment in segments:
        segment_name = f"{bag_name}_segment_{segment.index}"
        segment_output = output_dir / segment_name
        segment_bag_path = segment_bags_dir / f"{segment_name}.bag"
        duration = segment.end_time - segment.start_time

        segment_info = {
            "source_bag": str(bag_path),
            "segment_index": segment.index,
            "segment_name": segment_name,
            "duration_sec": duration,
        }

        # Check if already exists
        if skip_existing and segment_output.exists() and (segment_output / "times.txt").exists():
            print(f"  Skipped (exists): {segment_name}")
            segment_info["skipped"] = True
            results.append(segment_info)
            continue

        # Extract segment bag (delete existing if present)
        if segment_bag_path.exists():
            segment_bag_path.unlink()
        msg_count = extract_segment_bag(bag_path, segment_bag_path, segment)
        segment_info["segment_messages"] = msg_count

        # Extract images from segment
        try:
            extraction_stats = extractor.extract_images(segment_bag_path, segment_output)
            segment_info["frame_count"] = extraction_stats["frame_count"]
            print(f"  Created: {segment_name} (duration: {duration:.2f}s, {extraction_stats['frame_count']} frames)")
        except Exception as e:
            print(f"  Error extracting {segment_name}: {e}")
            segment_info["error"] = str(e)

        results.append(segment_info)

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Segment ROS bags and extract synchronized images in one pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Process all bags with RealSense D435i
    python rosbag_segment_extract.py -i input_dir/ -o output_dir/ realsense_d435i

    # Process with compressed images
    python rosbag_segment_extract.py -i input_dir/ -o output_dir/ oak --compressed

    # Resume processing (skip existing)
    python rosbag_segment_extract.py -i input_dir/ -o output_dir/ realsense_d435i --skip-existing

    # With custom minimum duration and log file
    python rosbag_segment_extract.py -i input_dir/ -o output_dir/ realsense_d435i --min-duration 3.0 --log log.json
        """
    )
    parser.add_argument("-i", "--input", required=True, help="Input directory containing ROS bag files")
    parser.add_argument("-o", "--output", required=True, help="Output directory for extracted data")
    parser.add_argument("camera_type", choices=["oak", "realsense_d435i"], help="Camera type")
    parser.add_argument("--pattern", default="*.bag", help="Glob pattern for bag files (default: *.bag)")
    parser.add_argument("--compressed", action="store_true", help="Image topics are compressed")
    parser.add_argument("--min-duration", type=float, default=5.0, help="Minimum segment duration in seconds (default: 5.0)")
    parser.add_argument("--skip-existing", action="store_true", help="Skip segments that already exist")
    parser.add_argument("--log", help="Path to save processing log as JSON")

    args = parser.parse_args()

    input_dir = Path(args.input)
    output_dir = Path(args.output)

    # Validate input
    if not input_dir.exists():
        print(f"Error: Input directory does not exist: {input_dir}")
        sys.exit(1)

    # Find bag files
    bag_files = sorted(input_dir.glob(args.pattern))
    if not bag_files:
        print(f"No bag files found matching '{args.pattern}' in {input_dir}")
        sys.exit(1)

    print(f"Found {len(bag_files)} bag file(s)")
    print(f"Camera type: {args.camera_type}")
    print(f"Compressed: {args.compressed}")
    print(f"Min duration: {args.min_duration}s")
    print(f"Output directory: {output_dir}")

    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)

    # Create extractor
    extractor = ImageExtractor(args.camera_type, args.compressed)

    # Process all bags
    all_results = []
    start_time = datetime.now()

    for bag_path in bag_files:
        results = process_bag(
            bag_path, output_dir, extractor,
            min_duration=args.min_duration,
            skip_existing=args.skip_existing
        )
        all_results.extend(results)

    # Summary
    elapsed = (datetime.now() - start_time).total_seconds()
    total_frames = sum(r.get("frame_count", 0) for r in all_results if "frame_count" in r)

    print(f"\n{'='*60}")
    print(f"Summary:")
    print(f"  Input bags: {len(bag_files)}")
    print(f"  Total segments: {len(all_results)}")
    print(f"  Total frames extracted: {total_frames}")
    print(f"  Output directory: {output_dir}")
    print(f"  Elapsed time: {elapsed:.2f}s")

    # Save log
    if args.log:
        log_data = {
            "processed_at": datetime.now().isoformat(),
            "input_directory": str(input_dir),
            "output_directory": str(output_dir),
            "camera_type": args.camera_type,
            "compressed": args.compressed,
            "min_duration": args.min_duration,
            "total_bags": len(bag_files),
            "total_segments": len(all_results),
            "total_frames": total_frames,
            "elapsed_seconds": elapsed,
            "segments": all_results,
        }
        with open(args.log, 'w') as f:
            json.dump(log_data, f, indent=2)
        print(f"  Log saved to: {args.log}")


if __name__ == "__main__":
    main()
