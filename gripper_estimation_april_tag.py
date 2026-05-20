"""
Gripper Distance Estimation via AprilTag Detection

Detects left/right gripper AprilTags in image sequences to estimate
gripper opening distance over time. The pixel distance between tag centers
is normalized to [0, 1] and saved to disk along with a plot.

Supported camera configs live in configs/ as JSON files.
Use --configs to select one (defaults to D405).
"""

import json
import os
import cv2 as cv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import glob
import copy
from pupil_apriltags import Detector
import argparse
from tqdm import tqdm

import numpy as np

# Default config path relative to this script
DEFAULT_CONFIG = "configs/sroi_v2_d405.json"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Estimate gripper distance from AprilTag detections in image sequences."
    )
    parser.add_argument("data_folder", help="Path to the image sequence folder")
    parser.add_argument(
        "--configs",
        default=DEFAULT_CONFIG,
        help=f"Path to camera config JSON (default: {DEFAULT_CONFIG})",
    )
    return parser.parse_args()


def load_config(path):
    """Load and print camera configuration from a JSON file."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Config file not found: {path}")
    with open(path) as f:
        cfg = json.load(f)
    return cfg


def print_config(cfg, path):
    """Print a summary of the loaded configuration."""
    desc = cfg.get("description", "N/A")
    print(f"Config file : {path}")
    print(f"  description          : {desc}")
    print(f"  gripper_roi          : {cfg['gripper_roi']}")
    print(f"  tag_family           : {cfg['tag_family']}")
    print(f"  left_gripper_tag_id  : {cfg['left_gripper_tag_id']}")
    print(f"  right_gripper_tag_id : {cfg['right_gripper_tag_id']}")
    print(f"  norminal_diag_distance: {cfg['norminal_diag_distance']}")
    print(f"  threshold            : {cfg['threshold']}")


def check_tag_valid(tag, norminal_diag_distance=50, threshold=10):
    """Check whether a detected tag's diagonal distances are within expected range."""
    diag_distance_1 = np.linalg.norm(tag.corners[0] - tag.corners[2])
    diag_distance_2 = np.linalg.norm(tag.corners[1] - tag.corners[3])
    print(f"diag_distance_1: {diag_distance_1}, diag_distance_2: {diag_distance_2}, "
          f"norminal_diag_distance: {norminal_diag_distance}")
    if abs(diag_distance_1 - norminal_diag_distance) > threshold or \
       abs(diag_distance_2 - norminal_diag_distance) > threshold:
        return False
    return True


def detect_april_tag(image, gripper_roi, at_detector, left_gripper_tag_id,
                     right_gripper_tag_id, norminal_diag_distance, threshold):
    """Detect left and right gripper AprilTags and return the pixel distance between their centers."""
    left_gripper_tag = None
    right_gripper_tag = None

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    gray[0:gripper_roi, :] = 0
    ret = at_detector.detect(gray)
    print("detected tag: ", ret)
    for res in ret:
        if res.tag_id == left_gripper_tag_id:
            if check_tag_valid(res, norminal_diag_distance=norminal_diag_distance,
                               threshold=threshold):
                left_gripper_tag = res
        elif res.tag_id == right_gripper_tag_id:
            if check_tag_valid(res, norminal_diag_distance=norminal_diag_distance,
                               threshold=threshold):
                right_gripper_tag = res

    if left_gripper_tag is not None and right_gripper_tag is not None:
        gripper_distance = abs(right_gripper_tag.center[0] - left_gripper_tag.center[0])
        return gripper_distance
    return None


def find_distances_max_min(distances, threshold=0.05, threshold_num=10):
    """Find robust max/min distances by requiring multiple measurements near the extremum."""
    distances_copy = copy.deepcopy(distances)
    ret_max_distance, ret_min_distance = None, None
    while True:
        valid_max = False
        max_dist_id = np.argmax(distances)
        max_distance = distances[max_dist_id]
        data_in_range = distances[
            (distances > max_distance * (1 - threshold)) &
            (distances < max_distance * (1 + threshold))
        ]
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
        data_in_range = distances[
            (distances > min_distance * (1 - threshold)) &
            (distances < min_distance * (1 + threshold))
        ]
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


def save_gripper_distance_plot(gripper_distances_normalized, data_folder):
    """Save a time vs normalized gripper distance plot."""
    plt.figure(figsize=(12, 6))
    time_indices = np.arange(len(gripper_distances_normalized))
    plt.plot(time_indices, gripper_distances_normalized, 'b-', linewidth=1.5,
             label='Normalized Gripper Distance')
    plt.xlabel('Frame Index (Time)')
    plt.ylabel('Normalized Gripper Distance')
    plt.title('Time vs Gripper Distance')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plot_path = data_folder + "gripper_distances.jpg"
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close()
    print("Saved gripper distance plot to", plot_path)


def main(data_folder, config_path):
    if not data_folder.endswith('/'):
        data_folder += '/'

    # Load config
    cfg = load_config(config_path)
    print("=" * 50)
    print_config(cfg, config_path)
    print("=" * 50)

    gripper_roi = cfg["gripper_roi"]
    tag_family = cfg["tag_family"]
    left_gripper_tag_id = cfg["left_gripper_tag_id"]
    right_gripper_tag_id = cfg["right_gripper_tag_id"]
    norminal_diag_distance = cfg["norminal_diag_distance"]
    threshold = cfg["threshold"]

    at_detector = Detector(
        families=tag_family,
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    image_nums = len(glob.glob(data_folder + f"color_*.png"))
    print("image_nums: ", image_nums)
    gripper_distances = []
    for idx in tqdm(range(image_nums)):
        color_image_path = data_folder + f"color_{idx:06d}.png"
        color_image = cv.imread(color_image_path)
        gripper_distance = detect_april_tag(
            color_image, gripper_roi, at_detector,
            left_gripper_tag_id, right_gripper_tag_id,
            norminal_diag_distance, threshold,
        )
        gripper_distances.append(gripper_distance)

    gripper_distance_valid = [x for x in gripper_distances if x is not None]
    max_gripper_distance = max(gripper_distance_valid)
    min_gripper_distance = min(gripper_distance_valid)
    print("max_gripper_distance: ", max_gripper_distance)
    print("min_gripper_distance: ", min_gripper_distance)
    gripper_distances_processed = copy.copy(gripper_distances)

    for idx, gripper_distance in enumerate(gripper_distances_processed):
        if idx == 0:
            if gripper_distance is None:
                gripper_distances_processed[idx] = max_gripper_distance
        elif gripper_distance is None:
            gripper_distances_processed[idx] = gripper_distances_processed[idx - 1]

    gripper_distances_normalized = np.asarray(gripper_distances_processed)

    filtered_max_distance, filtered_min_distance = find_distances_max_min(
        gripper_distances_normalized, threshold=0.05, threshold_num=10,
    )

    gripper_distances_normalized = (
        (gripper_distances_normalized - filtered_min_distance)
        / (filtered_max_distance - filtered_min_distance)
    )
    gripper_distances_normalized[gripper_distances_normalized > 1] = 1
    gripper_distances_normalized[gripper_distances_normalized < 0] = 0

    np.savetxt(data_folder + "gripper_distances.txt", gripper_distances_normalized)
    print("Finished processing gripper distances to file", data_folder + "gripper_distances.txt")

    save_gripper_distance_plot(gripper_distances_normalized, data_folder)


if __name__ == "__main__":
    args = parse_args()
    print(f"data_folder: {args.data_folder}")
    main(args.data_folder, args.configs)
