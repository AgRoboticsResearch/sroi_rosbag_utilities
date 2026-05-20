#!/usr/bin/env python3
"""
Batch Gripper Estimation using AprilTags (RealSense D405 defaults).

D405-specific version with tuned parameters:
  gripper_roi=200, nominal_diag=42, threshold=15

Usage:
    python gripper_estimation_batch_d405.py -i test_output/
    python gripper_estimation_batch_d405.py -i test_output/ --skip-existing
"""

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

import cv2 as cv
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for headless operation
import matplotlib.pyplot as plt
import glob
import copy
from pupil_apriltags import Detector
from tqdm import tqdm

import numpy as np


def check_tag_valid(tag, norminal_diag_distance=50, threshold=10):
    """Check if detected tag is valid based on diagonal distance."""
    diag_distance_1 = np.linalg.norm(tag.corners[0] - tag.corners[2])
    diag_distance_2 = np.linalg.norm(tag.corners[1] - tag.corners[3])
    if abs(diag_distance_1 - norminal_diag_distance) > threshold or abs(diag_distance_2 - norminal_diag_distance) > threshold:
        return False
    else:
        return True


def detect_april_tag(image, gripper_roi, at_detector, left_gripper_tag_id, right_gripper_tag_id, norminal_diag_distance, threshold, verbose=False):
    """Detect AprilTags and compute gripper distance."""
    gripper_distance_current = None
    left_gripper_tag = None
    right_gripper_tag = None

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    gray[0:gripper_roi, :] = 0
    ret = at_detector.detect(gray)
    if verbose:
        print("detected tag: ", ret)
    for res in ret:
        if res.tag_id == left_gripper_tag_id:
            if check_tag_valid(res, norminal_diag_distance=norminal_diag_distance, threshold=threshold):
                left_gripper_tag = res
        elif res.tag_id == right_gripper_tag_id:
            if check_tag_valid(res, norminal_diag_distance=norminal_diag_distance, threshold=threshold):
                right_gripper_tag = res

    if left_gripper_tag is not None and right_gripper_tag is not None:
        gripper_distance = abs(right_gripper_tag.center[0] - left_gripper_tag.center[0])
        return gripper_distance
    else:
        return None


def find_distances_max_min(distances, threshold=0.05, threshold_num=10):
    """Find valid max and min distances from the data."""
    distances_copy = copy.deepcopy(distances)
    ret_max_distance, ret_min_distance = None, None
    # Max
    while True:
        valid_max = False
        max_dist_id = np.argmax(distances)
        max_distance = distances[max_dist_id]
        data_in_range = distances[(distances > max_distance * (1 - threshold)) & (distances < max_distance * (1 + threshold))]
        num_data_in_range = len(data_in_range)
        if num_data_in_range < threshold_num:
            pass
        else:
            valid_max = True
        if not valid_max:
            distances[max_dist_id] = -1
        if valid_max:
            ret_max_distance = max_distance
            break
    distances = distances_copy
    while True:
        valid_min = False
        min_dist_id = np.argmin(distances)
        min_distance = distances[min_dist_id]
        data_in_range = distances[(distances > min_distance * (1 - threshold)) & (distances < min_distance * (1 + threshold))]
        num_data_in_range = len(data_in_range)
        if num_data_in_range < threshold_num:
            pass
        else:
            valid_min = True
        if not valid_min:
            distances[min_dist_id] = np.inf
        if valid_min:
            ret_min_distance = min_distance
            break
    return ret_max_distance, ret_min_distance


def save_gripper_distance_plot(gripper_distances_normalized, output_path):
    """
    Create and save a time vs gripper distance plot.

    Args:
        gripper_distances_normalized (np.array): Normalized gripper distances
        output_path (Path): Path to save the plot
    """
    plt.figure(figsize=(12, 6))
    time_indices = np.arange(len(gripper_distances_normalized))
    plt.plot(time_indices, gripper_distances_normalized, 'b-', linewidth=1.5, label='Normalized Gripper Distance')
    plt.xlabel('Frame Index (Time)')
    plt.ylabel('Normalized Gripper Distance')
    plt.title('Time vs Gripper Distance')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()


def process_folder(data_folder: Path, at_detector, args) -> dict:
    """
    Process a single folder to estimate gripper distances.

    Returns:
        Dictionary with processing results
    """
    result = {
        "folder": str(data_folder),
        "status": "success",
        "frame_count": 0,
        "valid_detections": 0,
    }

    # Check for color images
    color_images = sorted(data_folder.glob("color_*.png"))
    if not color_images:
        result["status"] = "skipped"
        result["error"] = "No color images found"
        return result

    # Check if already processed
    output_file = data_folder / "gripper_distances.txt"
    if args.skip_existing and output_file.exists():
        result["status"] = "skipped"
        result["error"] = "Already exists"
        return result

    try:
        image_nums = len(color_images)
        result["frame_count"] = image_nums

        gripper_distances = []
        for idx in tqdm(range(image_nums), desc=f"  Frames", leave=False):
            color_image_path = data_folder / f"color_{idx:06d}.png"
            color_image = cv.imread(str(color_image_path))
            if color_image is None:
                gripper_distances.append(None)
                continue
            gripper_distance = detect_april_tag(
                color_image, args.gripper_roi, at_detector,
                args.left_tag_id, args.right_tag_id,
                args.nominal_diag, args.threshold,
                verbose=args.verbose
            )
            gripper_distances.append(gripper_distance)

        gripper_distance_valid = [x for x in gripper_distances if x is not None]
        result["valid_detections"] = len(gripper_distance_valid)

        if not gripper_distance_valid:
            result["status"] = "error"
            result["error"] = "No valid AprilTag detections"
            return result

        max_gripper_distance = max(gripper_distance_valid)
        min_gripper_distance = min(gripper_distance_valid)

        gripper_distances_processed = copy.copy(gripper_distances)
        for idx, gripper_distance in enumerate(gripper_distances_processed):
            if idx == 0:
                if gripper_distance is None:
                    gripper_distances_processed[idx] = max_gripper_distance
            elif gripper_distance is None:
                gripper_distances_processed[idx] = gripper_distances_processed[idx - 1]

        gripper_distances_normalized = np.asarray(gripper_distances_processed)

        filtered_max_distance, filtered_min_distance = find_distances_max_min(
            gripper_distances_normalized, threshold=0.05, threshold_num=10
        )

        gripper_distances_normalized = (gripper_distances_normalized - filtered_min_distance) / (
            filtered_max_distance - filtered_min_distance
        )
        gripper_distances_normalized[gripper_distances_normalized > 1] = 1
        gripper_distances_normalized[gripper_distances_normalized < 0] = 0

        # Save results
        np.savetxt(output_file, gripper_distances_normalized)

        # Save plot
        plot_path = data_folder / "gripper_distances.jpg"
        save_gripper_distance_plot(gripper_distances_normalized, plot_path)

        result["max_distance"] = max_gripper_distance
        result["min_distance"] = min_gripper_distance

    except Exception as e:
        result["status"] = "error"
        result["error"] = str(e)

    return result


def find_data_folders(input_dir: Path) -> list:
    """Find all folders that contain color images."""
    data_folders = []
    for item in sorted(input_dir.iterdir()):
        if item.is_dir() and item.name != "segment_bags":
            # Check if folder has color images
            color_images = list(item.glob("color_*.png"))
            if color_images:
                data_folders.append(item)
    return data_folders


def main():
    parser = argparse.ArgumentParser(
        description="Batch process gripper estimation using AprilTags",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Process all folders in test_output/
    python gripper_estimation_batch.py -i test_output/

    # Skip already processed folders
    python gripper_estimation_batch.py -i test_output/ --skip-existing

    # Custom AprilTag parameters
    python gripper_estimation_batch.py -i test_output/ --left-tag-id 0 --right-tag-id 15
        """
    )
    parser.add_argument("-i", "--input", required=True, help="Input directory containing data folders")
    parser.add_argument("--skip-existing", action="store_true", help="Skip folders that already have gripper_distances.txt")
    parser.add_argument("--gripper-roi", type=int, default=200, help="ROI for gripper detection (default: 200)")
    parser.add_argument("--left-tag-id", type=int, default=0, help="AprilTag ID for left gripper (default: 0)")
    parser.add_argument("--right-tag-id", type=int, default=15, help="AprilTag ID for right gripper (default: 15)")
    parser.add_argument("--nominal-diag", type=int, default=42, help="Nominal diagonal distance for tag validation (default: 42)")
    parser.add_argument("--threshold", type=int, default=15, help="Threshold for tag validation (default: 15)")
    parser.add_argument("--verbose", action="store_true", help="Print detailed detection info")
    parser.add_argument("--log", help="Path to save processing log as JSON")

    args = parser.parse_args()

    input_dir = Path(args.input)
    if not input_dir.exists():
        print(f"Error: Input directory does not exist: {input_dir}")
        sys.exit(1)

    # Find data folders
    data_folders = find_data_folders(input_dir)
    if not data_folders:
        print(f"No data folders with color images found in {input_dir}")
        sys.exit(1)

    print(f"Found {len(data_folders)} data folder(s)")

    # Create AprilTag detector
    at_detector = Detector(
        families="tag16h5",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    # Process all folders
    all_results = []
    start_time = datetime.now()

    for folder in tqdm(data_folders, desc="Processing folders"):
        print(f"\n{folder.name}:")
        result = process_folder(folder, at_detector, args)
        all_results.append(result)

        if result["status"] == "success":
            print(f"  Done: {result['frame_count']} frames, {result['valid_detections']} valid detections")
        elif result["status"] == "skipped":
            print(f"  Skipped: {result.get('error', '')}")
        else:
            print(f"  Error: {result.get('error', '')}")

    # Summary
    elapsed = (datetime.now() - start_time).total_seconds()
    success_count = sum(1 for r in all_results if r["status"] == "success")
    skipped_count = sum(1 for r in all_results if r["status"] == "skipped")
    error_count = sum(1 for r in all_results if r["status"] == "error")
    total_frames = sum(r.get("frame_count", 0) for r in all_results if r["status"] == "success")

    print(f"\n{'=' * 60}")
    print(f"Summary:")
    print(f"  Total folders: {len(data_folders)}")
    print(f"  Successful: {success_count}")
    print(f"  Skipped: {skipped_count}")
    print(f"  Errors: {error_count}")
    print(f"  Total frames processed: {total_frames}")
    print(f"  Elapsed time: {elapsed:.2f}s")

    # Save log
    if args.log:
        import json
        log_data = {
            "processed_at": datetime.now().isoformat(),
            "input_directory": str(input_dir),
            "total_folders": len(data_folders),
            "successful": success_count,
            "skipped": skipped_count,
            "errors": error_count,
            "total_frames": total_frames,
            "elapsed_seconds": elapsed,
            "results": all_results,
        }
        with open(args.log, 'w') as f:
            json.dump(log_data, f, indent=2)
        print(f"  Log saved to: {args.log}")


if __name__ == "__main__":
    main()
