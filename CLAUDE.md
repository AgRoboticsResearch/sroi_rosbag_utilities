# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This repository contains utilities for processing ROS bag files from robotic manipulation experiments (SROI - Stereo RGB-based manipulation). The pipeline extracts robot trajectories, camera images, and gripper states from ROS bags and converts them to formats suitable for training robot learning policies (e.g., LeRobot datasets).

### Data Pipeline Architecture

The repository implements a multi-stage processing pipeline:

1. **Segmentation** (`rosbag_segment.py`) - Extracts action segments from ROS bags based on `/upi/status/is_action` topic
2. **Image Extraction** (`extract_stereo_rosbags.py`) - Extracts stereo/RGB images and camera info from segmented bags
3. **Gripper Estimation** (`gripper_estimation_april_tag.py`) - Detects AprilTags to estimate gripper state
4. **Trajectory Processing** (`transform_trajectory.py`) - Transforms ORB-SLAM camera trajectories to be relative to initial frame
5. **Dataset Conversion** (`lerobot/sroi_to_lerobot.py`) - Converts processed data to LeRobot dataset format

## Common Development Commands

### Installing Dependencies
```bash
pip install -r requirements.txt
```

Key dependencies: `rosbags`, `opencv-python`, `numpy`, `PyYAML`, `pupil_apriltags`

### Running the Full Pipeline

Process a single ROS bag from start to finish:

```bash
# Step 1: Segment the bag (extract action periods)
python rosbag_segment.py input.bag -o /path/to/output

# Step 2: Extract stereo/RGB images
python3 extract_stereo_rosbags.py /path/to/segment.bag /output/folder/ realsense_d435i --compressed

# Step 3: Estimate gripper distances from AprilTags
python gripper_estimation_april_tag.py /output/folder/

# Step 4: Generate ORB-SLAM YAML config
python3 create_orb_slam_yaml.py /output/folder/ realsense_d435i

# Step 5: Transform trajectory (after running ORB-SLAM)
python transform_trajectory.py /output/folder/

# Step 6: Convert to LeRobot dataset (run from lerobot directory)
cd /path/to/lerobot
python /path/to/sroi_rosbag_utilities/lerobot/sroi_to_lerobot.py \
    --data_path "/output/folder/" \
    --repo_id "username/dataset_name" \
    --fps 30
```

## Key Scripts and Their Roles

### Core Processing Scripts

- **`rosbag_segment.py`** - Segments ROS bags where `/upi/status/is_action` is true. Supports single files or directories with glob patterns.
- **`extract_stereo_rosbags.py`** - Extracts synchronized stereo/RGB images using the modern `rosbags` library. Supports multiple camera types (`realsense_d435i`, `oak`). Handles both compressed and raw image topics.
- **`gripper_estimation_april_tag.py`** - Detects AprilTags in images to estimate gripper opening distance. Uses `pupil_apriltags` library.
- **`transform_trajectory.py`** - Transforms ORB-SLAM camera trajectories from world coordinates to be relative to the initial camera frame.
- **`create_orb_slam_yaml.py`** - Generates ORB-SLAM configuration YAML files from camera info JSONs.

### LeRobot Integration

- **`lerobot/sroi_to_lerobot.py`** - Main converter from SROI format to LeRobot datasets. Handles trajectory data (4x4 transforms), gripper states, and camera images.

**Important**: This script must be run from the LeRobot directory due to import dependencies. It changes the working directory internally and requires LeRobot to be in the Python path.

### extract_rgbd/ Directory

Contains additional extraction utilities:
- **`extract_rgbd/extract_stereo_rosbags.py`** - Alternative stereo extraction (older version using `rosbag` library)
- **`extract_rgbd/extract_rgbd.py`** - RGB-D extraction utilities
- **`extract_rgbd/msgReaders.py`** - ROS message reader utilities

## Camera Topic Mappings

The scripts support different camera types with specific topic conventions:

### RealSense D435i
- Left infrared: `/camera/infra1/image_rect_raw`
- Right infrared: `/camera/infra2/image_rect_raw`
- Color: `/camera/color/image_raw`
- Depth: `/camera/aligned_depth_to_color/image_raw`

### OAK
- Left: `/oak/left/image_rect_color`
- Right: `/oak/right/image_rect_color`
- Stereo depth: `/oak/stereo/image_raw`

## Coordinate System Notes

- Trajectory data is stored as 3x4 transformation matrices (first 3 rows of 4x4 homogeneous transforms)
- The fourth row `[0, 0, 0, 1]` is appended to create full 4x4 matrices
- ORB-SLAM trajectories are in world coordinates; `transform_trajectory.py` converts to camera-relative coordinates
- Euler angles use ZYX convention (yaw, pitch, roll) for LeRobot compatibility

## ORB_SLAM3 Subdirectory

The repository includes a complete ORB_SLAM3 build in `ORB_SLAM3/`. This is a third-party visual SLAM system used for camera trajectory estimation. Treat this as a dependency - modifications should generally be avoided unless specifically working on SLAM functionality.

## Data Format Requirements

For LeRobot conversion, input directories must contain:
- `CameraTrajectoryTransformed.txt` - Trajectory with timestamps (format: `timestamp r00 r01... r23`)
- `gripper_distances.txt` - One gripper distance value per line
- `color_XXXXXX.png` - Camera images with zero-padded indices

## Working with LeRobot

When running LeRobot-related scripts:
- The script may change directories to `/home/hls/codes/lerobot` (hardcoded path)
- Ensure LeRobot is installed and accessible in Python path
- Check `lerobot/example_convert.py` for usage patterns
