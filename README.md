# ROS Bag Segmenter

A Python utility for segmenting ROS bag files based on action status messages. This tool extracts segments from ROS bag files where the `/upi/status/is_action` topic is `true`.


## Usage

The script can process either a single ROS bag file or a directory containing multiple bag files.

### Basic Usage

Process a single bag file:
```bash
python rosbag_segment.py input.bag
```

Process all bag files in a directory:
```bash
python rosbag_segment.py /path/to/directory
```

### Advanced Options

- `-o, --output`: Specify output directory for segmented files
- `-t, --template`: Custom filename template for output files
- `-p, --pattern`: Pattern to match bag files (default: *.bag)

### Examples

1. Process a single file with custom output directory:
```bash
python rosbag_segment.py input.bag -o /path/to/output
```

2. Process all bag files in a directory with custom output:
```bash
python rosbag_segment.py /path/to/directory -o /path/to/output
```

3. Use custom filename template:
```bash
python rosbag_segment.py input.bag -o /path/to/output -t "custom_segment_{}.bag"
```

4. Process specific bag files using a pattern:
```bash
python rosbag_segment.py /path/to/directory -p "oak_d_sr_*.bag"
```

5. Complete example with all options:
```bash
python rosbag_segment.py /path/to/directory -o /path/to/output -t "custom_segment_{}.bag" -p "oak_d_sr_*.bag"
```

### Output

The script will create segmented bag files with the following naming convention:
- If no template is specified: `{original_filename}_segment_{number}.bag`
- If template is specified: Uses the provided template with `{}` replaced by the segment number


## Notes

- The script looks for the `/upi/status/is_action` topic in the bag files
- Segments are created based on continuous `true` values in the action status
- Progress and results are displayed in the console during processing
- The script automatically creates the output directory if it doesn't exist


# Full Post Processing

Complete workflow for processing ROS bag files from segmentation to gripper estimation:

### Step 1: Segment ROS Bags
```bash
python rosbag_segment.py input.bag -o /path/to/output
```

### Step 2: Extract Images
```bash
python3 extract_stereo_rosbags.py /path/to/bag.bag /output/folder/ realsense_d435i --compressed
```

### Step 3: Extract Gripper
```bash
python gripper_estimation_april_tag.py /output/folder/
```

### Step 4: Extract Images
```bash
python3 create_orb_slam_yaml.py /output/folder/ realsense_d435i
```
