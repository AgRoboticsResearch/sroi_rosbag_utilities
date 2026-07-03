# SROI ROS Bag Utilities

Tools for recording, processing, and converting SROI robot data for ORB-SLAM and LeRobot training pipelines.

---

## Pipeline A: Record → LeRobot (Direct Recording)

Record directly from a RealSense camera and convert to a LeRobot training dataset.

### Step 1: Record

```bash
# PNG mode (default, lossless)
python record_realsense.py -o ~/Desktop/435 --camera realsense_d405

# JPEG mode (smaller files)
python record_realsense.py -o ~/Desktop/435 --camera realsense_d405 --image-format jpeg

# MP4 mode (smallest, auto-encodes after each episode)
python record_realsense.py -o ~/Desktop/435 --camera realsense_d405 --encode-video

# Headless SSH session (no display needed)
python record_realsense.py -o ~/Desktop/435 --headless

# Combine: headless + MP4
python record_realsense.py -o ~/Desktop/435 --headless --encode-video --camera realsense_d405
```

Controls (GUI window or terminal in headless): `r` record, `s` stop, `q` quit

Session folders are named `{timestamp}-{format}` (e.g. `20260520_143052-mp4`).

### Step 2: Decode MP4 (only if recorded with --encode-video)

```bash
python decode_videos.py ~/Desktop/435/20260520_143052-mp4 --recursive
```

Produces a sibling folder with `-png` postfix containing PNG images for ORB-SLAM.

### Step 3: Run ORB-SLAM3

Note: The ORB-SLAM YAML is generated automatically during recording for both D405 (fixed calibration) and D435i (from live calibration).

```bash
# Single episode
cd ~/code/ORB_SLAM3
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt \
    ~/Desktop/435/20260520_143052-png/episode_001/orb_slam_realsense_d405.yaml \
    ~/Desktop/435/20260520_143052-png/episode_001 false

# Batch all episodes
./batches/orbslam_batch_d405.sh ~/Desktop/435/20260520_143052-png
```

### Step 4: Transform Trajectory

```bash
# Single episode
python transform_trajectory.py ~/Desktop/435/20260520_143052-png/episode_001

# All episodes recursively
python transform_trajectory.py --recursive ~/Desktop/435/20260520_143052-png
```

### Step 5: Extract Gripper

```bash
# Default: D405 config
python gripper_estimation_april_tag.py ~/Desktop/435/20260520_143052-png/episode_001

# D435 config
python gripper_estimation_april_tag.py /output/folder/ --configs configs/sroi_v1_d435.json

# All episodes in a session share one min/max, robustly detected from the pooled data
python gripper_estimation_april_tag.py --recursive ~/Desktop/435/20260520_143052-png
```

Gripper distance is normalized to `[0, 1]`. With `--recursive`, every episode in that run
is normalized against one shared min/max (robust clustered-extremum over pooled raw
distances) instead of each episode picking its own range. To set the range manually
instead of auto-detecting it, add `"gripper_min_distance"` / `"gripper_max_distance"`
(both required) to the camera config JSON passed via `--configs`.

Each run also writes `gripper_distance_distribution.png` next to the input path — a
histogram of the pooled raw distances with the selected min/max marked, so you can
sanity-check the range before trusting the normalized `gripper_distances.txt` files.

Add `--median_filter 3` (must be odd) to smooth single-frame AprilTag detection noise
in the raw distances before the min/max is computed and episodes are normalized.
Default is `1` (no filtering).

### Step 6: QC and Convert to LeRobot Dataset

For a single decoded session folder:

```bash
python lerobot/sroi_to_lerobot.py \
    --data_path ~/Desktop/435/20260520_143052-png \
    --repo_id sroi/lab_picking \
    --fps 30 \
    --root /mnt/data0/data/sroi/sroi_lab_picking \
    --task "pick the red strawberry"
```

For a full recording day containing many `*-png` session folders, first run QC,
then use the QC CSV to keep only selected categories (usually `ok`) while
converting all sessions into one LeRobot dataset:

```bash
# Writes /path/to/day/qc.csv and /path/to/day/qc.pdf
python visualization/qc.py /path/to/day -o /path/to/day/qc

python lerobot/sroi_to_lerobot.py \
    --data_path /path/to/day \
    --repo_id sroi/lab_picking \
    --fps 30 \
    --root /mnt/data0/data/sroi/sroi_lab_picking \
    --task "pick the red strawberry" \
    --qc_csv /path/to/day/qc.csv \
    --qc_categories ok \
    --num_workers 4
```

`--root` must point to a dataset directory that does not already exist;
`LeRobotDataset.create()` creates it fresh.

### Optional: Merge LeRobot Datasets

Use the official LeRobot edit tool when combining multiple completed local
datasets into one dataset. The official checkout currently needs the `py312`
conda environment and its source tree on `PYTHONPATH`:

```bash
cd /home/zfei/code/lerobots/lerobot_official

PYTHONPATH=/home/zfei/code/lerobots/lerobot_official/src \
/home/zfei/anaconda3/envs/py312/bin/python -m lerobot.scripts.lerobot_edit_dataset \
    --new_repo_id sroi/sroi_v2_merged_official \
    --new_root /mnt/data1/data/lerobot/sroi_v2_merged_official \
    --operation.type merge \
    --operation.repo_ids "['sroi/lerobot_sroi_v2', 'sroi/sroi_v2_20260611']" \
    --operation.roots "['/mnt/data1/data/lerobot/lerobot_sroi_v2', '/mnt/data1/data/lerobot/sroi_v2_20260611']"
```

`--operation.repo_ids` labels the input datasets; `--operation.roots` provides
the exact local dataset folders. `--new_root` must not already exist. Keep a
small provenance note (for example `MERGE_INFO.md` and `merge_info.json`) in the
merged dataset folder listing the input datasets, frame counts, and merge
command.

---

## Pipeline B: ROS Bag → LeRobot (Legacy)

Process pre-recorded ROS bag files.

### Step 1: Segment ROS Bags

```bash
python rosbag_segment.py input.bag -o /path/to/output
```

### Step 2: Extract Images from Bags

```bash
python3 extract_stereo_rosbags.py /path/to/bag.bag /output/folder/ realsense_d435i --compressed
```

### Step 3: Create ORB-SLAM Config (D435i from bags only)

```bash
python3 create_orb_slam_yaml.py /output/folder/ realsense_d435i
```

### Step 4: Extract Gripper

```bash
python gripper_estimation_april_tag.py /output/folder/
```

Then continue with ORB-SLAM3 (Step 3 in Pipeline A) and follow remaining steps.

---

## Dependencies

```bash
pip install pyrealsense2 opencv-python numpy tqdm pupil-apriltags scipy PyYAML
pip install av  # for --encode-video and decode_videos.py
pip install rosbags  # for extract_stereo_rosbags.py (Pipeline B)
pip install lerobot  # for sroi_to_lerobot.py
pip install plotly matplotlib  # for visualization tools
```
