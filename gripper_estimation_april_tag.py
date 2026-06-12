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
from pathlib import Path

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
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Interactive calibration mode with GUI sliders",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output config path for --calibrate (default: overwrite --configs)",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Process all episode subdirectories recursively",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print per-frame detection diagnostics",
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
    print(f"  nominal_diag_distance: {cfg.get('nominal_diag_distance', cfg.get('norminal_diag_distance', 'N/A'))}")
    print(f"  threshold            : {cfg['threshold']}")


def check_tag_valid(tag, nominal_diag_distance=50, threshold=10, verbose=False):
    """Check whether a detected tag's diagonal distances are within expected range."""
    diag_distance_1 = np.linalg.norm(tag.corners[0] - tag.corners[2])
    diag_distance_2 = np.linalg.norm(tag.corners[1] - tag.corners[3])
    if verbose:
        print(f"diag_distance_1: {diag_distance_1}, diag_distance_2: {diag_distance_2}, "
              f"nominal_diag_distance: {nominal_diag_distance}")
    if abs(diag_distance_1 - nominal_diag_distance) > threshold or \
       abs(diag_distance_2 - nominal_diag_distance) > threshold:
        return False
    return True


def detect_april_tag(image, gripper_roi, at_detector, left_gripper_tag_id,
                     right_gripper_tag_id, nominal_diag_distance, threshold,
                     gripper_roi_bottom=None, verbose=False):
    """Detect left and right gripper AprilTags and return the pixel distance between their centers."""
    left_gripper_tag = None
    right_gripper_tag = None

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    gray[0:gripper_roi, :] = 0
    if gripper_roi_bottom is not None:
        gray[gripper_roi_bottom:, :] = 0
    ret = at_detector.detect(gray)
    if verbose:
        print("detected tag: ", ret)
    for res in ret:
        if res.tag_id == left_gripper_tag_id:
            if check_tag_valid(res, nominal_diag_distance=nominal_diag_distance,
                               threshold=threshold, verbose=verbose):
                left_gripper_tag = res
        elif res.tag_id == right_gripper_tag_id:
            if check_tag_valid(res, nominal_diag_distance=nominal_diag_distance,
                               threshold=threshold, verbose=verbose):
                right_gripper_tag = res

    if left_gripper_tag is not None and right_gripper_tag is not None:
        gripper_distance = abs(right_gripper_tag.center[0] - left_gripper_tag.center[0])
        return gripper_distance
    return None


def find_distances_max_min(distances, threshold=0.05, threshold_num=10):
    """Find robust max/min distances by requiring multiple measurements near the extremum."""
    distances_copy = copy.deepcopy(distances)
    ret_max_distance, ret_min_distance = None, None
    for _ in range(len(distances)):
        max_dist_id = np.argmax(distances)
        max_distance = distances[max_dist_id]
        data_in_range = distances[
            (distances > max_distance * (1 - threshold)) &
            (distances < max_distance * (1 + threshold))
        ]
        if len(data_in_range) >= threshold_num:
            ret_max_distance = max_distance
            break
        distances[max_dist_id] = -1
    if ret_max_distance is None:
        ret_max_distance = np.max(distances_copy)

    distances = distances_copy
    for _ in range(len(distances)):
        min_dist_id = np.argmin(distances)
        min_distance = distances[min_dist_id]
        data_in_range = distances[
            (distances > min_distance * (1 - threshold)) &
            (distances < min_distance * (1 + threshold))
        ]
        if len(data_in_range) >= threshold_num:
            ret_min_distance = min_distance
            break
        distances[min_dist_id] = np.inf
    if ret_min_distance is None:
        ret_min_distance = np.min(distances_copy)

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


def draw_detections(image, results, left_id, right_id, gripper_roi_top, gripper_roi_bottom):
    """Draw detected tags on image with color-coded outlines and IDs."""
    vis = image.copy()
    # Draw ROI lines
    cv.line(vis, (0, gripper_roi_top), (vis.shape[1], gripper_roi_top), (0, 255, 255), 1)
    cv.line(vis, (0, gripper_roi_bottom), (vis.shape[1], gripper_roi_bottom), (0, 255, 255), 1)
    for tag in results:
        is_left = tag.tag_id == left_id
        is_right = tag.tag_id == right_id
        color = (0, 255, 0) if is_left else (0, 0, 255) if is_right else (128, 128, 128)
        pts = tag.corners.astype(int).reshape(-1, 1, 2)
        cv.polylines(vis, [pts], True, color, 2)
        cx, cy = int(tag.center[0]), int(tag.center[1])
        cv.circle(vis, (cx, cy), 4, color, -1)
        cv.putText(vis, f"id={tag.tag_id}", (cx - 20, cy - 10),
                    cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return vis


TAG_FAMILIES = ["tag16h5", "tag25h9", "tag36h11", "tagCircle21h7", "tagCircle49h12",
                "tagCustom48h12", "tagStandard41h12", "tagStandard52h13"]


def calibrate(data_folder, config_path, output_path):
    """Interactive calibration mode with cv2 GUI."""
    cfg = load_config(config_path)
    print_config(cfg, config_path)

    # Pick a sample image (middle frame for better coverage)
    images = sorted(glob.glob(os.path.join(data_folder, "color_*.png")))
    if not images:
        images = sorted(glob.glob(os.path.join(data_folder, "color_*.jpg")))
    if not images:
        print("No color images found in", data_folder)
        return
    sample_path = images[len(images) // 2]
    print(f"Using sample image: {sample_path}")

    # Terminal prompts for string/ID values
    current_family = cfg.get("tag_family", "tag16h5")
    print(f"\nAvailable tag families: {TAG_FAMILIES}")
    print(f"Current: {current_family}")
    choice = input("Enter tag family (press Enter to keep current): ").strip()
    tag_family = choice if choice in TAG_FAMILIES else current_family

    current_left = cfg.get("left_gripper_tag_id", 0)
    current_right = cfg.get("right_gripper_tag_id", 15)
    left_id = int(input(f"Left gripper tag ID [{current_left}]: ").strip() or current_left)
    right_id = int(input(f"Right gripper tag ID [{current_right}]: ").strip() or current_right)

    # Read sample image
    image = cv.imread(sample_path)
    h, w = image.shape[:2]

    default_bottom = cfg.get("gripper_roi_bottom", h)

    # Create window and trackbars
    win = "AprilTag Calibration"
    cv.namedWindow(win, cv.WINDOW_NORMAL)

    cv.createTrackbar("top roi", win, cfg["gripper_roi"], h, lambda x: None)
    cv.createTrackbar("bottom roi", win, default_bottom, h, lambda x: None)
    cv.createTrackbar("diag dist", win, cfg.get("nominal_diag_distance", cfg.get("norminal_diag_distance", 50)), 200, lambda x: None)
    cv.createTrackbar("threshold", win, cfg["threshold"], 100, lambda x: None)

    detector = Detector(families=tag_family, nthreads=1, quad_decimate=1.0,
                        quad_sigma=0, refine_edges=1, decode_sharpening=0.25, debug=0)

    print("\nSliders: top roi | bottom roi | diag dist | threshold")
    print("Keys: 's' save config | 'q' quit")
    while True:
        top_roi = cv.getTrackbarPos("top roi", win)
        bottom_roi = cv.getTrackbarPos("bottom roi", win)
        norminal_diag = cv.getTrackbarPos("diag dist", win)
        threshold_val = cv.getTrackbarPos("threshold", win)

        # Detect
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        gray_masked = gray.copy()
        gray_masked[0:top_roi, :] = 0
        gray_masked[bottom_roi:, :] = 0
        results = detector.detect(gray_masked)

        # Draw
        vis = draw_detections(image, results, left_id, right_id, top_roi, bottom_roi)

        # Info overlay
        left_found = any(t.tag_id == left_id for t in results)
        right_found = any(t.tag_id == right_id for t in results)
        ok = left_found and right_found
        y0 = 0
        info_lines = [
            (f"family={tag_family}  left_id={left_id}  right_id={right_id}", (200, 200, 200)),
            (f"top={top_roi}  bottom={bottom_roi}  diag={norminal_diag}  thresh={threshold_val}", (200, 200, 200)),
            (f"Detected: {len(results)} tags", (200, 200, 200)),
            (f"Left(id={left_id}): {'FOUND' if left_found else 'MISSING'}", (0, 255, 0) if left_found else (0, 0, 255)),
            (f"Right(id={right_id}): {'FOUND' if right_found else 'MISSING'}", (0, 255, 0) if right_found else (0, 0, 255)),
        ]
        if ok:
            left_tag = next(t for t in results if t.tag_id == left_id)
            right_tag = next(t for t in results if t.tag_id == right_id)
            dist = abs(right_tag.center[0] - left_tag.center[0])
            info_lines.append((f"Gripper dist: {dist:.1f}px", (255, 255, 0)))

        for text, color in info_lines:
            y0 += 30
            cv.putText(vis, text, (10, y0), cv.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv.imshow(win, vis)
        key = cv.waitKey(100) & 0xFF
        if key == ord('s'):
            new_cfg = {
                "description": cfg.get("description", ""),
                "gripper_roi": top_roi,
                "gripper_roi_bottom": bottom_roi,
                "tag_family": tag_family,
                "left_gripper_tag_id": left_id,
                "right_gripper_tag_id": right_id,
                "nominal_diag_distance": norminal_diag,
                "threshold": threshold_val,
            }
            out = output_path or config_path
            with open(out, 'w') as f:
                json.dump(new_cfg, f, indent=4)
            print(f"Config saved to {out}")
            print(json.dumps(new_cfg, indent=2))
            break
        elif key == ord('q'):
            print("Quit without saving.")
            break

    cv.destroyAllWindows()


def main(data_folder, config_path, verbose=False):
    if not data_folder.endswith('/'):
        data_folder += '/'

    # Load config
    cfg = load_config(config_path)
    print("=" * 50)
    print_config(cfg, config_path)
    print("=" * 50)

    gripper_roi = cfg["gripper_roi"]
    gripper_roi_bottom = cfg.get("gripper_roi_bottom")
    tag_family = cfg["tag_family"]
    left_gripper_tag_id = cfg["left_gripper_tag_id"]
    right_gripper_tag_id = cfg["right_gripper_tag_id"]
    nominal_diag_distance = cfg.get("nominal_diag_distance", cfg.get("norminal_diag_distance", 50))
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

    images = sorted(glob.glob(data_folder + "color_*.png") + glob.glob(data_folder + "color_*.jpg"))
    image_nums = len(images)
    print("image_nums: ", image_nums)
    gripper_distances = []
    for color_image_path in tqdm(images):
        color_image = cv.imread(color_image_path)
        gripper_distance = detect_april_tag(
            color_image, gripper_roi, at_detector,
            left_gripper_tag_id, right_gripper_tag_id,
            nominal_diag_distance, threshold,
            gripper_roi_bottom=gripper_roi_bottom,
            verbose=verbose,
        )
        gripper_distances.append(gripper_distance)

    gripper_distance_valid = [x for x in gripper_distances if x is not None]
    if not gripper_distance_valid:
        print(f"Error: no valid AprilTag detections in {data_folder}. Skipping.")
        return
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
    data_path = args.data_folder
    if args.calibrate:
        calibrate(data_path, args.configs, args.output)
    elif args.recursive:
        dirs = sorted(d for d in Path(data_path).rglob("episode_*")
                      if d.is_dir())
        if not dirs:
            print(f"No episode directories found under {data_path}")
        print(f"Processing {len(dirs)} episode(s)")
        for d in dirs:
            print(f"\n--- {d.name} ---")
            main(str(d) + '/', args.configs, verbose=args.verbose)
    else:
        main(data_path, args.configs, verbose=args.verbose)
