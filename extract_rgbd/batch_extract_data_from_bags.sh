#!/bin/bash

# Check if the correct number of arguments are provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <file_pattern> <output_dir> <camera_optical_pose_topic>"
    echo "Example: $0 '/ldata/data/spi/z1_calib/z1_rs_calib_lab_2025-01-22-08-*' '/ldata/data/temp/spi_postproc/' '/camera_optical_pose_from_tf'"
    exit 1
fi

# Assign arguments to variables
file_pattern="$1"
output_dir="$2"
camera_optical_pose_topic="$3"

# Loop through all files matching the pattern
for bag_path in $file_pattern; do
    # Check if the file exists (in case the pattern matches no files)
    if [ -e "$bag_path" ]; then
        echo "Processing file: $bag_path"
        
        # Run the first Python script
        python extract_stereo_ros1.py "$bag_path" "$output_dir" realsense_d435i --compressed
        
        # Run the second Python script
        python extract_endpose_ros1.py "$bag_path" "$output_dir" "$camera_optical_pose_topic"
    else
        echo "No files found matching the pattern: $file_pattern"
    fi
done