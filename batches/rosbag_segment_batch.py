#!/usr/bin/env python3
"""
Batch ROS Bag Segmenter

Enhanced version for batch processing multiple ROS bag files with better organization and logging.

Usage:
    # Process all bags in a directory
    python rosbag_segment_batch.py brl_lab_sroi_rosbags_20260206/ -o segmented_output/

    # Process with custom pattern and organize by source file
    python rosbag_segment_batch.py brl_lab_sroi_rosbags_20260206/ -o segmented_output/ --organize-by-source

    # Dry run to see what will be processed
    python rosbag_segment_batch.py brl_lab_sroi_rosbags_20260206/ --dry-run
"""

import rosbag
from rosbag import Bag
import argparse
import os
import glob
import json
from datetime import datetime
from pathlib import Path


def extract_true_segments(input_file, output_template=None):
    """Extract segments from a single bag file where /upi/status/is_action is true."""
    # Read all status messages
    status_data = []
    try:
        with Bag(input_file, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=['/upi/status/is_action']):
                status_data.append((t.to_sec(), msg.data))
    except Exception as e:
        print(f"  Error reading {input_file}: {e}")
        return 0, []

    # Detect continuous true segments
    segments = []
    current_segment = None
    for timestamp, value in status_data:
        if value:  # Currently true
            if not current_segment:  # New segment starts
                current_segment = {
                    'start': timestamp,
                    'end': timestamp,
                }
            else:  # Continue current segment
                current_segment['end'] = timestamp
        else:  # Currently false
            if current_segment:  # End current segment
                segments.append(current_segment)
                current_segment = None

    # Handle last segment
    if current_segment:
        segments.append(current_segment)

    # Split original bag file
    output_files = []
    for i, seg in enumerate(segments):
        if output_template is None:
            base_name = os.path.splitext(os.path.basename(input_file))[0]
            output_file = f"{base_name}_segment_{i+1}.bag"
        else:
            output_file = output_template.format(i+1)

        print(f"  Processing segment {i+1}: {seg['start']:.3f} to {seg['end']:.3f} (duration: {seg['end']-seg['start']:.3f}s)")

        try:
            with Bag(input_file, 'r') as inbag:
                with Bag(output_file, 'w') as outbag:
                    # Iterate through all messages
                    for topic, msg, t in inbag.read_messages():
                        msg_time = t.to_sec()
                        if seg['start'] <= msg_time <= seg['end']:
                            outbag.write(topic, msg, t)
            output_files.append({
                'segment': i+1,
                'file': output_file,
                'start': seg['start'],
                'end': seg['end'],
                'duration': seg['end'] - seg['start']
            })
        except Exception as e:
            print(f"  Error writing segment {i+1}: {e}")

    return len(segments), output_files


def process_bag_file(input_file, output_dir=None, template=None, organize_by_source=False):
    """Process a single bag file and return results."""
    base_name = os.path.splitext(os.path.basename(input_file))[0]

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

        if organize_by_source:
            # Create subdirectory for each source bag
            source_output_dir = os.path.join(output_dir, base_name)
            os.makedirs(source_output_dir, exist_ok=True)
            output_template = os.path.join(source_output_dir, f"{base_name}_segment_{{}}.bag")
        else:
            # All segments in same output directory
            if template:
                output_template = os.path.join(output_dir, template)
            else:
                output_template = os.path.join(output_dir, f"{base_name}_segment_{{}}.bag")
    else:
        output_template = template

    print(f"\n{'='*70}")
    print(f"Processing: {input_file}")
    print(f"Source file size: {os.path.getsize(input_file) / (1024**3):.2f} GB")
    print(f"{'='*70}")

    num_segments, output_files = extract_true_segments(input_file, output_template)

    result = {
        'source_file': input_file,
        'base_name': base_name,
        'num_segments': num_segments,
        'output_files': output_files,
        'status': 'success' if num_segments > 0 else 'no_segments'
    }

    print(f"  ✓ Found {num_segments} segments in {base_name}")
    return result


def batch_process_bag_files(input_dir, output_dir=None, pattern='*.bag', organize_by_source=False,
                            dry_run=False, log_file=None, verbose=False):
    """Process multiple bag files in a directory."""
    # Validate input directory
    if not os.path.isdir(input_dir):
        print(f"Error: Input path '{input_dir}' does not exist or is not a directory")
        return None

    # Find all bag files
    search_pattern = os.path.join(input_dir, pattern)
    bag_files = sorted(glob.glob(search_pattern))

    if not bag_files:
        print(f"No bag files found matching pattern '{pattern}' in {input_dir}")
        return None

    print(f"\n{'='*70}")
    print(f"BATCH ROS BAG SEGMENTER")
    print(f"{'='*70}")
    print(f"Input directory: {input_dir}")
    print(f"Output directory: {output_dir if output_dir else 'Same as input'}")
    print(f"File pattern: {pattern}")
    print(f"Organize by source: {organize_by_source}")
    print(f"Found {len(bag_files)} bag files to process")
    if verbose:
        print(f"Verbose mode enabled")
    print(f"{'='*70}")

    if dry_run:
        print("\n[DRY RUN] Would process the following files:")
        for i, bag_file in enumerate(bag_files, 1):
            size_gb = os.path.getsize(bag_file) / (1024**3)
            print(f"  {i}. {os.path.basename(bag_file)} ({size_gb:.2f} GB)")
        print(f"\nTotal: {len(bag_files)} files")
        return {'dry_run': True, 'files': bag_files}

    # Process each file
    results = []
    total_segments = 0
    start_time = datetime.now()

    for i, bag_file in enumerate(bag_files, 1):
        print(f"\n[{i}/{len(bag_files)}] Processing {os.path.basename(bag_file)}...")
        result = process_bag_file(bag_file, output_dir, organize_by_source=organize_by_source)
        results.append(result)
        total_segments += result['num_segments']
        if verbose and result['output_files']:
            for seg in result['output_files']:
                print(f"    Segment {seg['segment']}: {seg['file']} ({seg['duration']:.3f}s)")

    # Generate summary
    end_time = datetime.now()
    duration = (end_time - start_time).total_seconds()

    summary = {
        'start_time': start_time.isoformat(),
        'end_time': end_time.isoformat(),
        'duration_seconds': duration,
        'total_files': len(bag_files),
        'total_segments': total_segments,
        'output_directory': output_dir,
        'results': results
    }

    # Print summary
    print(f"\n{'='*70}")
    print(f"BATCH PROCESSING COMPLETE")
    print(f"{'='*70}")
    print(f"Total files processed: {len(bag_files)}")
    print(f"Total segments created: {total_segments}")
    print(f"Total time: {duration:.2f} seconds ({duration/60:.1f} minutes)")
    print(f"Output directory: {output_dir}")
    print(f"{'='*70}\n")

    # Save log file if specified
    if log_file:
        with open(log_file, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"Log saved to: {log_file}")

    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Batch extract true segments from ROS bag files.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process all bags in directory
  python rosbag_segment_batch.py brl_lab_sroi_rosbags_20260206/ -o segmented_output/

  # Organize output by source file
  python rosbag_segment_batch.py brl_lab_sroi_rosbags_20260206/ -o segmented_output/ --organize-by-source

  # Dry run to preview
  python rosbag_segment_batch.py brl_lab_sroi_rosbags_20260206/ --dry-run

  # With log file
  python rosbag_segment_batch.py brl_lab_sroi_rosbags_20260206/ -o segmented_output/ --log process_log.json
        """
    )

    parser.add_argument('input', type=str, help='Input directory containing bag files')
    parser.add_argument('-o', '--output', type=str, help='Output directory for segmented bag files')
    parser.add_argument('-p', '--pattern', type=str, default='*.bag', help='Pattern to match bag files (default: *.bag)')
    parser.add_argument('--organize-by-source', action='store_true', help='Create subdirectories for each source bag file')
    parser.add_argument('--dry-run', action='store_true', help='Show what would be processed without actually processing')
    parser.add_argument('--log', type=str, help='Save processing log to JSON file')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')

    args = parser.parse_args()

    # Run batch processing
    result = batch_process_bag_files(
        input_dir=args.input,
        output_dir=args.output,
        pattern=args.pattern,
        organize_by_source=args.organize_by_source,
        dry_run=args.dry_run,
        log_file=args.log,
        verbose=args.verbose
    )
