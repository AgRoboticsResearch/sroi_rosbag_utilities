# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

Utilities for processing ROS bag files from robotic manipulation experiments (SROI - Stereo RGB-based manipulation). The pipeline extracts robot trajectories, camera images, and gripper states from ROS bags and converts them to LeRobot datasets for training robot learning policies.

## Python Environment

Always activate the conda environment `sroi_rosbags` before running any Python scripts:

```bash
conda activate sroi_rosbags
pip install -r requirements.txt  # first-time setup
```

## Data Pipeline

The pipeline processes ROS bags through 6 stages. You can either run the unified pipeline or individual scripts.

### Unified Pipeline (`run_pipeline.py`)

```bash
# Full pipeline (ORB-SLAM step must be run manually, see below)
python run_pipeline.py -i input_bags/ -o output/ --camera realsense_d435i --dry-run

# Specific steps only
python run_pipeline.py -i input_bags/ -o output/ --steps segment,gripper,transform

# With LeRobot conversion
python run_pipeline.py -i bags/ -o output/ --lerobot-repo user/dataset --lerobot-task "pick object"
```

Available steps: `segment`, `gripper`, `orbslam`, `transform`, `visualize`, `lerobot`

### Individual Pipeline Scripts

Each stage has both a single-file script and a batch processing variant:

1. **Segment + Extract** - `rosbag_segment_extract.py` (combined) or separate `rosbag_segment.py` + `extract_stereo_rosbags.py`
2. **Gripper Estimation** - `gripper_estimation_april_tag.py` (single) or `gripper_estimation_batch.py` (batch)
3. **ORB-SLAM** - Run manually inside Docker (see below)
4. **Trajectory Transform** - `transform_trajectory.py` (supports `--recursive` for batch)
5. **Visualization** - `batch_visualize_trajectory.py`
6. **LeRobot Conversion** - `lerobot/sroi_to_lerobot.py`

### ORB-SLAM (Docker Workflow)

ORB-SLAM3 runs inside a Docker container with GPU access. It cannot be automated by `run_pipeline.py`:

```bash
# 1. Enter Docker container
cd ORB_SLAM3 && ./run_docker.sh

# 2. Inside Docker, run batch processing:
./orbslam_batch.sh /codes/sroi_rosbag_utilities/output/ true false
#                      ^input_dir              ^skip_existing ^visualization
```

The batch script looks for `*_segment_*/` folders containing `left_*.png` and `right_*.png` images, and produces `CameraTrajectory.txt` in each.

## LeRobot Conversion

**Important**: `lerobot/sroi_to_lerobot.py` must be run from the LeRobot source directory because it imports `lerobot` as a package. The script hardcodes a sys.path to `/home/zfei/code/lerobot/src` — adjust this for your setup.

```bash
cd /path/to/lerobot
python /path/to/sroi_rosbag_utilities/lerobot/sroi_to_lerobot.py \
    --data_path "/output/folder/" \
    --repo_id "username/dataset_name" \
    --fps 30 \
    --episodes all
```

### Multi-episode support

- Pass a **parent directory** containing multiple episode subdirectories and use `--episodes all` (default) or `--episodes ep1,ep2`
- Pass a **single episode directory** for backward compatibility (auto-detected by presence of `CameraTrajectoryTransformed.txt`)

### Dataset output format

- **State** (7D): `[x, y, z, wx, wy, wz, gripper_pos]` — position + rotation vector + gripper
- **Action** (7D): next frame's state (absolute, not delta)
- **Images**: `color_*.png` stored as video in LeRobot format

### Merging datasets

`lerobot/merge_datasets.py` merges multiple LeRobot datasets (checks FPS, robot_type, and feature compatibility):

```bash
python lerobot/merge_datasets.py --datasets ds1 ds2 --output merged --root /path/to/datasets
```

## Camera Topic Mappings

| Camera Type | Left | Right | Color | Depth |
|---|---|---|---|---|
| `realsense_d435i` | `/camera/infra1/image_rect_raw` | `/camera/infra2/image_rect_raw` | `/camera/color/image_raw` | `/camera/aligned_depth_to_color/image_raw` |
| `oak` | `/oak/left/image_rect_color` | `/oak/right/image_rect_color` | — | `/oak/stereo/image_raw` |

Use `--compressed` flag when extracting images from bags that have compressed image topics.

## Coordinate System

- Trajectory data stored as 3x4 transformation matrices (first 3 rows of 4x4 homogeneous transforms); the fourth row `[0, 0, 0, 1]` is appended during processing
- ORB-SLAM outputs world-coordinate trajectories; `transform_trajectory.py` converts to camera-relative coordinates
- Rotation representations: rotation vectors (axis-angle `wx, wy, wz`) in the LeRobot converter, used via `lerobot.utils.rotation.Rotation`

## Data Format Requirements

For LeRobot conversion, each episode directory must contain:
- `CameraTrajectoryTransformed.txt` — timestamp + 12 floats per line (3x4 matrix)
- `gripper_distances.txt` — one float per line
- `color_XXXXXX.png` — zero-padded indexed color images

## ORB_SLAM3 Subdirectory

Complete ORB_SLAM3 build included as a dependency. Do not modify unless specifically working on SLAM functionality.
