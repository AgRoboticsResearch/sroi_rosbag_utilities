# SROI to LeRobot Dataset Converter

This script converts robot end-effector trajectory data from SROI format to LeRobot dataset format for machine learning training.

## Features

- Converts 4x4 transformation matrices to end-effector deltas (position + orientation)
- Processes gripper distance data
- Handles camera images (color_*.png)
- Creates LeRobot-compatible datasets with proper video encoding
- Action space: `["delta_x_ee", "delta_y_ee", "delta_z_ee", "delta_roll_ee", "delta_pitch_ee", "delta_yaw_ee"]`

## Data Format Expected

Your data directory should contain:
```
data_directory/
├── CameraTrajectoryTransformed.txt  # End-effector trajectory (timestamp + 3x4 transformation matrix)
├── gripper_distances.txt            # Gripper distance values (one per line)
├── color_000000.png                 # Camera images
├── color_000001.png
└── ...
```

## Usage

### Basic Usage

```bash
cd /home/zfei/codes/lerobot
python /path/to/sroi_to_lerobot.py \
    --data_path "/mnt/ldata/data/spi/spi/rs435_2025-07-23-09-10-02_sb_lab_picking/postproc/rs435_2025-07-23-09-10-02_segment_1/" \
    --repo_id "your_username/your_dataset_name" \
    --fps 30 \
    --root "/tmp/lerobot_datasets" \
    --task "Robot end-effector manipulation task"
```

### Push to Hugging Face Hub

```bash
cd /home/zfei/codes/lerobot
python /path/to/sroi_to_lerobot.py \
    --data_path "/your/data/path/" \
    --repo_id "your_username/your_dataset_name" \
    --push_to_hub \
    --task "Your task description"
```

### Quick Test

```bash
cd /home/zfei/codes/lerobot
python /home/zfei/codes/sroi/sroi_rosbag_utilities/lerobot/test_conversion.py
```

## Arguments

- `--data_path`: Path to SROI data directory (required)
- `--repo_id`: Dataset repository ID, e.g., "username/dataset_name" (required)
- `--fps`: Frames per second for the dataset (default: 30)
- `--root`: Root directory for dataset storage (default: current directory)
- `--push_to_hub`: Push dataset to Hugging Face Hub (flag)
- `--task`: Task description for the dataset (default: "End-effector manipulation task")

## Output Dataset Format

The created LeRobot dataset contains:

### Observations
- `observation.images.camera`: Camera images (CHW format)
- `observation.state`: 7D state vector [x, y, z, roll, pitch, yaw, gripper_distance]

### Actions
- `action`: 6D action vector [delta_x_ee, delta_y_ee, delta_z_ee, delta_roll_ee, delta_pitch_ee, delta_yaw_ee]

## Technical Details

### Coordinate System
- Uses rotation matrices from the trajectory file
- Converts to Euler angles (roll, pitch, yaw) using ZYX convention
- Computes deltas between consecutive frames
- Handles angle wrapping for orientation deltas

### Data Processing
1. Loads trajectory data and parses 3x4 transformation matrices
2. Converts to 4x4 matrices by appending [0, 0, 0, 1]
3. Extracts positions and converts rotation matrices to Euler angles
4. Computes frame-to-frame deltas for actions
5. Loads gripper distances and camera images
6. Creates LeRobot dataset with proper video encoding

### Error Handling
- Validates data path existence
- Ensures consistent data lengths across all modalities
- Handles missing images gracefully
- Provides detailed progress information

## Example Output

```
Converting data from: /your/data/path/
Creating dataset: your_username/your_dataset_name
Original trajectory shape: (125, 13)
Dataset length: 125 frames
Adding frames to dataset...
Processed 10/125 frames
...
Dataset created with 125 frames
Dataset episodes: 1
Conversion successful!
```

## Dependencies

- numpy
- PIL (Pillow)
- lerobot (LeRobot framework)
- pathlib

Make sure to run from the lerobot directory to ensure proper imports.
