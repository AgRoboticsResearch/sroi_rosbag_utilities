#!/bin/bash

# Define the base directory and command
base_dir="/codes/ORB_SLAM3"
command="./Examples/Stereo/stereo_kitti ./Vocabulary/ORBvoc.txt ./Examples/Stereo/RealSense_D435i.yaml"

# Check if a folder pattern argument is provided
if [[ -z "$1" ]]; then
  echo "Usage: $0 <folder_pattern>"
  echo "Please provide the folder pattern as an argument."
  exit 1
fi

# Get the folder pattern from the argument
folder_pattern="$1"

# Find all matching folders
folders=($folder_pattern)

# Iterate through each folder
for folder in "${folders[@]}"; do
  # Construct the full command
  full_command="${command} ${folder} false"

  # Change to the base directory
  cd "${base_dir}"

  # Execute the command
  echo "Running command in ${folder}:"
  echo "${full_command}"
  eval "${full_command}"

  # Optional: Check for errors (adjust as needed)
  if [ $? -ne 0 ]; then
    echo "Error occurred in ${folder}"
  fi
done