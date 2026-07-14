#!/bin/bash
# Batch process ORB_SLAM3 for multiple segment folders
#
# Usage (run inside Docker container):
#   ./orbslam_batch.sh /codes/sroi_rosbag_utilities/test_output/ true false
#
# Arguments:
#   $1 - input_directory: Path to directory containing segment folders
#   $2 - skip_existing: "true" to skip folders with CameraTrajectory.txt (default: false)
#   $3 - visualization: "true" to enable GUI (default: false)
#   $4/$5 - optional: --mask-config PATH

ORB_SLAM_DIR="/ORB_SLAM3"
STEREO_KITTI="$ORB_SLAM_DIR/Examples/Stereo/stereo_kitti"
VOCAB="$ORB_SLAM_DIR/Vocabulary/ORBvoc.txt"
CAMERA_CONFIG="Examples/Stereo/RealSense_D435i.yaml"

# Parse arguments
INPUT_DIR="${1:-}"
SKIP_EXIST="${2:-false}"
VISUALIZATION="${3:-false}"
MASK_CONFIG=""
if [ -n "${4:-}" ]; then
    if [ "$4" != "--mask-config" ] || [ -z "${5:-}" ] || [ -n "${6:-}" ]; then
        echo "Error: optional mask syntax is --mask-config PATH" >&2
        exit 1
    fi
    MASK_CONFIG="$5"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/orbslam_mask_helpers.sh"

if [ -z "$INPUT_DIR" ]; then
    echo "Usage: $0 <input_directory> [skip_existing] [visualization] [--mask-config PATH]"
    echo "  skip_existing: \"true\" to skip folders with CameraTrajectory.txt"
    echo "  visualization: \"true\" to enable GUI"
    echo "  --mask-config: optional gripper-mask JSON; omitted means no masking"
    exit 1
fi

if ! sroi_configure_mask "$MASK_CONFIG"; then
    exit 1
fi

# Remove trailing slash
INPUT_DIR="${INPUT_DIR%/}"

echo "============================================"
echo "ORB_SLAM3 Batch Processor"
echo "============================================"
echo "Input directory: $INPUT_DIR"
echo "Skip existing: $SKIP_EXIST"
echo "Visualization: $VISUALIZATION"
echo "Mask config: ${SROI_MASK_CONFIG:-none}"
echo ""

# Find segment folders
SEGMENT_FOLDERS=()
for dir in "$INPUT_DIR"/*_segment_*/; do
    [ -d "$dir" ] || continue
    dir="${dir%/}"  # Remove trailing slash
    # Check for left/right images (PNG or JPEG)
    if (ls "$dir"/left_*.png 1>/dev/null 2>&1 || ls "$dir"/left_*.jpg 1>/dev/null 2>&1) && \
       (ls "$dir"/right_*.png 1>/dev/null 2>&1 || ls "$dir"/right_*.jpg 1>/dev/null 2>&1); then
        SEGMENT_FOLDERS+=("$dir")
    fi
done

TOTAL_FOLDERS=${#SEGMENT_FOLDERS[@]}
echo "Found $TOTAL_FOLDERS segment folder(s)"

if [ "$TOTAL_FOLDERS" -eq 0 ]; then
    echo "No segment folders found with left/right images"
    exit 1
fi

# Process each folder
SUCCESS_COUNT=0
SKIP_COUNT=0
ERROR_COUNT=0

for folder in "${SEGMENT_FOLDERS[@]}"; do
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
    FOLDER_CONFIG="$folder/RealSense_D435i.yaml"
    if [ -f "$FOLDER_CONFIG" ]; then
        CONFIG_TO_USE="$FOLDER_CONFIG"
    else
        CONFIG_TO_USE="$CAMERA_CONFIG"
    fi

    if ! sroi_prepare_orb_input "$folder"; then
        echo "  Error: failed to prepare masked ORB input"
        ERROR_COUNT=$((ERROR_COUNT + 1))
        continue
    fi

    # Run ORB_SLAM3
    cd "$ORB_SLAM_DIR"

    if [ "$VISUALIZATION" = "true" ]; then
        "$STEREO_KITTI" "$VOCAB" "$CONFIG_TO_USE" "$SROI_ORB_INPUT_DIR" true
    else
        "$STEREO_KITTI" "$VOCAB" "$CONFIG_TO_USE" "$SROI_ORB_INPUT_DIR" false
    fi

    if sroi_collect_trajectory "$folder"; then
        echo "  Done: CameraTrajectory.txt saved"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    else
        echo "  Error: CameraTrajectory.txt not generated"
        ERROR_COUNT=$((ERROR_COUNT + 1))
    fi
    sroi_cleanup_mask_input
done

# Summary
echo ""
echo "============================================"
echo "Summary:"
echo "  Total folders: $TOTAL_FOLDERS"
echo "  Successful: $SUCCESS_COUNT"
echo "  Skipped: $SKIP_COUNT"
echo "  Errors: $ERROR_COUNT"
echo "============================================"
