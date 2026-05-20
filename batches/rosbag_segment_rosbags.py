#!/usr/bin/env python3
"""
ROS Bag Batch Segmenter using rosbags library.

Segments ROS1 bag files based on /upi/status/is_action topic,
extracting continuous True periods into separate bag files.

Usage:
    python rosbag_segment_rosbags.py -i input_dir/ -o output_dir/
    python rosbag_segment_rosbags.py -i input_dir/ -o output_dir/ --dry-run
    python rosbag_segment_rosbags.py -i input_dir/ -o output_dir/ --pattern "*.bag"
"""

import argparse
import json
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore


@dataclass
class Segment:
    """Represents a continuous segment where is_action is True."""
    start_time: float  # seconds
    end_time: float    # seconds
    index: int         # segment number within the bag


def find_action_segments(bag_path: Path) -> List[Segment]:
    """
    Find continuous segments where /upi/status/is_action is True.

    Args:
        bag_path: Path to the ROS bag file

    Returns:
        List of Segment objects with start/end times
    """
    segments = []
    typestore = get_typestore(Stores.ROS1_NOETIC)

    with Reader(bag_path) as reader:
        # Find the status topic connection
        status_conns = [c for c in reader.connections
                       if c.topic == '/upi/status/is_action']

        if not status_conns:
            print(f"  Warning: No /upi/status/is_action topic found in {bag_path.name}")
            return segments

        # Collect all status messages with timestamps
        status_messages = []
        for conn, timestamp, rawdata in reader.messages(status_conns):
            msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
            time_sec = timestamp / 1e9
            status_messages.append((time_sec, msg.data))

        if not status_messages:
            return segments

        # Sort by timestamp
        status_messages.sort(key=lambda x: x[0])

        # Find continuous True segments
        segment_start = None
        segment_index = 0

        for time_sec, is_action in status_messages:
            if is_action and segment_start is None:
                # Start of a new segment
                segment_start = time_sec
            elif not is_action and segment_start is not None:
                # End of current segment
                segments.append(Segment(
                    start_time=segment_start,
                    end_time=time_sec,
                    index=segment_index + 1
                ))
                segment_index += 1
                segment_start = None

        # Handle case where bag ends during an action
        if segment_start is not None:
            segments.append(Segment(
                start_time=segment_start,
                end_time=status_messages[-1][0],
                index=segment_index + 1
            ))

    return segments


def extract_segment(source_path: Path, output_path: Path, segment: Segment) -> int:
    """
    Extract a segment from source bag to a new bag file.

    Args:
        source_path: Path to source ROS bag
        output_path: Path for output bag file
        segment: Segment with start/end times

    Returns:
        Number of messages written
    """
    msg_count = 0

    with Reader(source_path) as reader:
        with Writer(output_path) as writer:
            # Copy all connections from source
            conn_map = {}
            for conn in reader.connections:
                new_conn = writer.add_connection(
                    conn.topic,
                    conn.msgtype,
                    msgdef=conn.msgdef.data,  # msgdef is a MessageDefinition object
                    md5sum=conn.digest,       # Reader uses 'digest', Writer uses 'md5sum'
                )
                conn_map[conn.id] = new_conn

            # Write messages within the segment time range
            for conn, timestamp, rawdata in reader.messages():
                time_sec = timestamp / 1e9
                if segment.start_time <= time_sec <= segment.end_time:
                    writer.write(conn_map[conn.id], timestamp, rawdata)
                    msg_count += 1

    return msg_count


def process_bag(bag_path: Path, output_dir: Path, dry_run: bool = False,
                skip_existing: bool = False, force: bool = False,
                min_duration: float = 5.0) -> List[dict]:
    """
    Process a single bag file, extracting all action segments.

    Args:
        bag_path: Path to the ROS bag file
        output_dir: Directory for output segment files
        dry_run: If True, don't actually extract segments
        skip_existing: If True, skip segments that already exist
        force: If True, overwrite existing segment files
        min_duration: Minimum segment duration in seconds (segments shorter than this are skipped)

    Returns:
        List of segment info dictionaries
    """
    bag_name = bag_path.stem  # filename without extension
    results = []

    print(f"\nProcessing: {bag_path.name}")

    # Find segments
    segments = find_action_segments(bag_path)

    if not segments:
        print(f"  No action segments found")
        return results

    # Filter out short segments
    if min_duration > 0:
        original_count = len(segments)
        segments = [s for s in segments if (s.end_time - s.start_time) >= min_duration]
        filtered_count = original_count - len(segments)
        if filtered_count > 0:
            print(f"  Filtered {filtered_count} segment(s) shorter than {min_duration}s")

    if not segments:
        print(f"  No segments after filtering")
        return results

    print(f"  Found {len(segments)} segment(s)")

    for segment in segments:
        output_name = f"{bag_name}_segment_{segment.index}.bag"
        output_path = output_dir / output_name
        duration = segment.end_time - segment.start_time

        segment_info = {
            "source_bag": str(bag_path),
            "segment_index": segment.index,
            "output_file": output_name,
            "start_time": segment.start_time,
            "end_time": segment.end_time,
            "duration_sec": duration,
        }

        if dry_run:
            print(f"  [DRY-RUN] Would create: {output_name} "
                  f"(duration: {duration:.2f}s)")
        elif output_path.exists() and skip_existing:
            print(f"  Skipped (exists): {output_name}")
            segment_info["skipped"] = True
        else:
            # Remove existing file if force is set
            if output_path.exists() and force:
                output_path.unlink()

            msg_count = extract_segment(bag_path, output_path, segment)
            segment_info["message_count"] = msg_count
            print(f"  Created: {output_name} "
                  f"(duration: {duration:.2f}s, "
                  f"{msg_count} messages)")

        results.append(segment_info)

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Batch segment ROS bag files based on /upi/status/is_action topic",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Process all bags in directory
    python rosbag_segment_rosbags.py -i input_dir/ -o output_dir/

    # Preview without creating files
    python rosbag_segment_rosbags.py -i input_dir/ -o output_dir/ --dry-run

    # Process only specific bags
    python rosbag_segment_rosbags.py -i input_dir/ -o output_dir/ --pattern "rs435_*.bag"

    # Save processing log
    python rosbag_segment_rosbags.py -i input_dir/ -o output_dir/ --log processing.json
        """
    )
    parser.add_argument(
        "-i", "--input",
        required=True,
        help="Input directory containing ROS bag files"
    )
    parser.add_argument(
        "-o", "--output",
        required=True,
        help="Output directory for segmented bag files"
    )
    parser.add_argument(
        "--pattern",
        default="*.bag",
        help="Glob pattern for bag files (default: *.bag)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview segments without creating files"
    )
    parser.add_argument(
        "--log",
        help="Path to save processing log as JSON"
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip segments that already exist in output directory"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing segment files"
    )
    parser.add_argument(
        "--min-duration",
        type=float,
        default=5.0,
        help="Minimum segment duration in seconds (default: 5.0, set to 0 to disable)"
    )

    args = parser.parse_args()

    input_dir = Path(args.input)
    output_dir = Path(args.output)

    # Validate input directory
    if not input_dir.exists():
        print(f"Error: Input directory does not exist: {input_dir}")
        sys.exit(1)

    if not input_dir.is_dir():
        print(f"Error: Input path is not a directory: {input_dir}")
        sys.exit(1)

    # Find bag files
    bag_files = sorted(input_dir.glob(args.pattern))

    if not bag_files:
        print(f"No bag files found matching pattern '{args.pattern}' in {input_dir}")
        sys.exit(1)

    print(f"Found {len(bag_files)} bag file(s)")

    # Create output directory
    if not args.dry_run:
        output_dir.mkdir(parents=True, exist_ok=True)
        print(f"Output directory: {output_dir}")
    else:
        print(f"[DRY-RUN] Would output to: {output_dir}")

    # Process all bags
    all_results = []
    start_time = datetime.now()

    for bag_path in bag_files:
        results = process_bag(bag_path, output_dir, args.dry_run,
                             args.skip_existing, args.force, args.min_duration)
        all_results.extend(results)

    # Summary
    elapsed = (datetime.now() - start_time).total_seconds()
    print(f"\n{'='*60}")
    print(f"Summary:")
    print(f"  Input bags: {len(bag_files)}")
    print(f"  Total segments: {len(all_results)}")

    if not args.dry_run:
        print(f"  Output directory: {output_dir}")
    else:
        print(f"  [DRY-RUN] No files created")

    print(f"  Elapsed time: {elapsed:.2f}s")

    # Save log if requested
    if args.log:
        log_data = {
            "processed_at": datetime.now().isoformat(),
            "input_directory": str(input_dir),
            "output_directory": str(output_dir),
            "pattern": args.pattern,
            "dry_run": args.dry_run,
            "total_bags": len(bag_files),
            "total_segments": len(all_results),
            "elapsed_seconds": elapsed,
            "segments": all_results,
        }
        with open(args.log, 'w') as f:
            json.dump(log_data, f, indent=2)
        print(f"  Log saved to: {args.log}")


if __name__ == "__main__":
    main()
