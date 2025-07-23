# Extract Stereo Images using rosbags Library

This script (`extract_stereo_rosbags.py`) is a modern replacement for `extract_stereo_ros1.py` that uses the `rosbags` library instead of the legacy `rosbag` library.

## Key Improvements

1. **Python 3 Native**: Works natively with Python 3 without ROS dependencies
2. **Better Performance**: The `rosbags` library is more efficient than the legacy `rosbag`
3. **No ROS Installation Required**: Doesn't require a full ROS installation
4. **Cross-Platform**: Works on Windows, macOS, and Linux
5. **Type Safety**: Better type annotations and error handling

## Installation

```bash
pip install -r requirements_rosbags.txt
```

## Usage

The script has the same interface as the original:

```bash
python3 extract_stereo_rosbags.py <bag_path> <save_folder> <camera_type> [--compressed] [--depth]
```

### Examples

For OAK camera:
```bash
python3 extract_stereo_rosbags.py /path/to/bag.bag /output/folder/ oak
```

For RealSense D435i camera with compressed images:
```bash
python3 extract_stereo_rosbags.py /path/to/bag.bag /output/folder/ realsense_d435i --compressed
```

## Supported Camera Types

- `oak`: OAK stereo camera
- `realsense_d435i`: Intel RealSense D435i camera

## Output

The script creates a folder with the bag name and saves:
- `left_XXXXXX.png`: Left camera images
- `right_XXXXXX.png`: Right camera images  
- `color_XXXXXX.png`: Color images (RealSense only)
- `camera_info_left.txt`: Left camera projection matrix
- `camera_info_right.txt`: Right camera projection matrix
- `camera_info_color.txt`: Color camera projection matrix (RealSense only)
- `times.txt`: Timestamp file

## Differences from Original Script

1. **Library**: Uses `rosbags` instead of `rosbag`
2. **Message Handling**: Direct deserialization of ROS messages
3. **Image Processing**: Native image processing without cv_bridge dependency
4. **Error Handling**: Better error messages and validation
5. **Code Structure**: Object-oriented design for better maintainability
