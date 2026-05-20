#!/bin/bash
# Batch process ORB_SLAM3 for D405 episode folders
#
# Usage (run inside Docker container):
#   ./orbslam_batch_d405.sh /codes/sroi_rosbag_utilities/recordings/1777549192/
#   ./orbslam_batch_d405.sh /codes/sroi_rosbag_utilities/recordings/1777549192/ true false
#
# Arguments:
#   $1 - session_dir: Path to session directory containing episode_* folders
#   $2 - skip_existing: "true" to skip episodes with CameraTrajectory.txt (default: true)
#   $3 - visualization: "true" to enable GUI (default: false)

ORB_SLAM_DIR="/ORB_SLAM3"
STEREO_KITTI="$ORB_SLAM_DIR/Examples/Stereo/stereo_kitti"
VOCAB="$ORB_SLAM_DIR/Vocabulary/ORBvoc.txt"

# Parse arguments
SESSION_DIR="${1:-}"
SKIP_EXIST="${2:-true}"
VISUALIZATION="${3:-false}"

if [ -z "$SESSION_DIR" ]; then
    echo "Usage: $0 <session_dir> [skip_existing] [visualization]"
    echo "  session_dir: path containing episode_* folders"
    echo "  skip_existing: \"true\" to skip episodes with CameraTrajectory.txt"
    echo "  visualization: \"true\" to enable GUI"
    exit 1
fi

# Remove trailing slash
SESSION_DIR="${SESSION_DIR%/}"

echo "============================================"
echo "ORB_SLAM3 Batch Processor (D405)"
echo "============================================"
echo "Session directory: $SESSION_DIR"
echo "Skip existing: $SKIP_EXIST"
echo "Visualization: $VISUALIZATION"
echo ""

# Find episode folders
EPISODE_FOLDERS=()
for dir in "$SESSION_DIR"/episode_*/; do
    [ -d "$dir" ] || continue
    dir="${dir%/}"
    # Check for left/right images
    if ls "$dir"/left_*.png 1>/dev/null 2>&1 && ls "$dir"/right_*.png 1>/dev/null 2>&1; then
        EPISODE_FOLDERS+=("$dir")
    fi
done

TOTAL_FOLDERS=${#EPISODE_FOLDERS[@]}
echo "Found $TOTAL_FOLDERS episode folder(s)"

if [ "$TOTAL_FOLDERS" -eq 0 ]; then
    echo "No episode folders found with left/right images"
    exit 1
fi

# Process each folder
SUCCESS_COUNT=0
SKIP_COUNT=0
ERROR_COUNT=0

for folder in "${EPISODE_FOLDERS[@]}"; do
    FOLDER_NAME=$(basename "$folder")
    OUTPUT_FILE="$folder/CameraTrajectory.txt"

    echo ""
    echo "Processing: $FOLDER_NAME"

    # Check if already processed
    if [ "$SKIP_EXIST" = "true" ] && [ -f "$OUTPUT_FILE" ]; then
        echo "  Skipped (already exists)"
        SKIP_COUNT=$((SKIP_COUNT + 1))
        continue
    fi

    # Check for camera config in folder
    FOLDER_CONFIG="$folder/orb_slam_realsense_d405.yaml"
    if [ -f "$FOLDER_CONFIG" ]; then
        CONFIG_TO_USE="$FOLDER_CONFIG"
    else
        echo "  Warning: $FOLDER_CONFIG not found, using default"
        CONFIG_TO_USE="Examples/Stereo/RealSense_D405.yaml"
    fi

    # Run ORB_SLAM3
    cd "$ORB_SLAM_DIR"

    if [ "$VISUALIZATION" = "true" ]; then
        "$STEREO_KITTI" "$VOCAB" "$CONFIG_TO_USE" "$folder" true
    else
        "$STEREO_KITTI" "$VOCAB" "$CONFIG_TO_USE" "$folder" false
    fi

    # Check if trajectory was created
    if [ -f "$OUTPUT_FILE" ]; then
        echo "  Done: CameraTrajectory.txt saved"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    else
        echo "  Error: CameraTrajectory.txt not generated"
        ERROR_COUNT=$((ERROR_COUNT + 1))
    fi
done

# Summary
echo ""
echo "============================================"
echo "Summary:"
echo "  Total episodes: $TOTAL_FOLDERS"
echo "  Successful: $SUCCESS_COUNT"
echo "  Skipped: $SKIP_COUNT"
echo "  Errors: $ERROR_COUNT"
echo "============================================"
