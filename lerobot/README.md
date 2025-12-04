# SROI to LeRobot Dataset Converter

This converter transforms SROI-format robot end-effector trajectory recordings into a LeRobot v3.0 compatible dataset.

The converter reads end-effector poses, gripper distances and per-frame camera images and produces a single episode that follows the LeRobot v3.0 feature contract used by the `so100`/EE pipelines.

## Highlights

- Produces LeRobot v3.0 datasets (features schema compatible with `ForwardKinematicsJointsToEE` / EE pipelines)
- Stores end-effector pose as rotation-vector (`ee.wx, ee.wy, ee.wz`) + translation (`ee.x, ee.y, ee.z`) and gripper position
- Saves `observation.images.camera` as video frames (CHW) and writes Parquet metadata
- Action and observation feature shapes match the recording example in `examples/so100_to_so100_EE/record.py`

## Data Layout Expected

Your data directory should contain at least:

```
data_directory/
├── CameraTrajectoryTransformed.txt   # timestamps + 3x4 transform per frame
├── gripper_distances.txt             # one gripper measurement per line
├── color_000000.png                  # camera images (color_*.png)
├── color_000001.png
└── ...
```

## Usage

Run the converter directly from anywhere (it will add the local `lerobot` package to `sys.path`):

```bash
python /home/zfei/code/sroi_rosbag_utilities/lerobot/sroi_to_lerobot.py \
    --data_path "/path/to/your/segment/" \
    --repo_id "your_username/your_dataset_name" \
    --fps 30 \
    --root "/tmp/lerobot_datasets" \
    --task "End-effector manipulation task"
```

To push the created dataset to the Hugging Face Hub add `--push_to_hub` and provide a `--repo_id` that you own:

```bash
python /home/zfei/code/sroi_rosbag_utilities/lerobot/sroi_to_lerobot.py \
    --data_path "/path/to/your/segment/" \
    --repo_id "your_username/your_dataset_name" \
    --push_to_hub \
    --task "Your task description"
```

There is also an example wrapper `example_convert.py` and a small `test_conversion.py` in the same folder for quick local testing.

## CLI Arguments

- `--data_path` (required): Path to the SROI segment folder
- `--repo_id` (required): Dataset repo id used for `LeRobotDataset.create` (e.g. `username/dataset`)
- `--fps`: Frames per second for the dataset (default: `30`)
- `--root`: Filesystem root where dataset is written (defaults to LeRobot internal default if omitted)
- `--push_to_hub`: If set, pushes the saved dataset to the HF Hub
- `--task`: Task description string stored with each frame

## Produced LeRobot v3.0 Feature Schema

The converter creates the following core features (matching the pipeline expectations in `record.py`):

- `observation.images.camera`:
    - `dtype`: `video`
    - `shape`: `(channels, height, width)` (CHW per-frame images)
    - `names`: `['channels', 'height', 'width']`

- `observation.state`:
    - `dtype`: `float32`
    - `shape`: `[7]`
    - `names`: `['ee.x','ee.y','ee.z','ee.wx','ee.wy','ee.wz','ee.gripper_pos']`

- `action`:
    - `dtype`: `float32`
    - `shape`: `[7]`
    - `names`: `['ee.x','ee.y','ee.z','ee.wx','ee.wy','ee.wz','ee.gripper_pos']`

Notes:

- The `ee.wx/wy/wz` fields store the rotation vector (axis-angle) representation (the same `rotvec` used in the LeRobot pipelines), not Euler angles. This matches the `ForwardKinematicsJointsToEE` and `InverseKinematicsEEToJoints` processors.
- `action` stores the *target* end-effector pose for the next timestep (so policies operating on EE-space can be trained directly).

## Processing Steps (high level)

1. Load `CameraTrajectoryTransformed.txt` and parse the per-frame 3x4 transforms.
2. Convert each 3x4 into a 4x4 pose matrix and extract translation (`ee.x, ee.y, ee.z`) and rotation as a rotation-vector (`ee.wx, ee.wy, ee.wz`).
3. Read `gripper_distances.txt` and align length with poses and images.
4. Load `color_*.png` images, convert to CHW and write them as video frames into the LeRobot dataset.
5. For each frame, create `observation.state` from the current pose+gripper and `action` from the next frame's pose+gripper (last frame repeats the final pose).
6. Save the episode (Parquet + video files) and optionally push to the hub.

## Error handling & validation

- The script validates that the `--data_path` exists and prints a clear error if not.
- It truncates arrays to the shortest modality (poses, gripper, images) and warns if images are missing.
- It reports progress during conversion and raises exceptions for unrecoverable issues.

## Example output

```
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

- `numpy`
- `Pillow` (PIL)
- `lerobot` (local codebase in `../lerobot`)

Run the converter from any location — the script will add the local `lerobot` package to `sys.path` so that the current repo is used.
