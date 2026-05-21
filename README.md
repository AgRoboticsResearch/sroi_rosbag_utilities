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

```bash
# Single episode
cd ~/code/ORB_SLAM3
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt \
    ~/Desktop/435/20260520_143052-png/episode_001/orb_slam_realsense_d405.yaml \
```

Note: The ORB-SLAM YAML is generated automatically during recording for both D405 (fixed calibration) and D435i (from live calibration).

```bash
# Batch all episodes
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
```

### Step 6: Convert to LeRobot Dataset

```bash
python lerobot/sroi_to_lerobot.py \
    --data_path ~/Desktop/435/20260520_143052-png \
    --repo_id sroi/lab_picking \
    --fps 30 \
    --root /mnt/data0/data/sroi/sroi_lab_picking \
    --task "pick the red strawberry"
```

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
pip install pyrealsense2 opencv-python numpy tqdm pupil-apriltags
pip install av  # for --encode-video and decode_videos.py
pip install lerobot  # for sroi_to_lerobot.py
```
