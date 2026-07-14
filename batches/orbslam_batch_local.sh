#!/bin/bash
# Batch process ORB_SLAM3 for episode folders
#
# Usage:
#   ./orbslam_batch_local.sh <session_dir> <orb_slam_dir> [skip_existing] [visualization] [--mask-config PATH]
#
# Examples:
#   ./orbslam_batch_local.sh /data/20260521_153031-png ~/code/ORB_SLAM3
#   ./orbslam_batch_local.sh /data/20260521_153031-png ~/code/ORB_SLAM3 true false
#   ./orbslam_batch_local.sh /data/20260521_153031-png ~/code/ORB_SLAM3 true false --mask-config configs/gripper_mask_sroi_v2_d405.json

SESSION_DIR="${1:-}"
ORB_SLAM_DIR="${2:-}"
SKIP_EXIST="${3:-true}"
VISUALIZATION="${4:-false}"
MASK_CONFIG=""
if [ -n "${5:-}" ]; then
    if [ "$5" != "--mask-config" ] || [ -z "${6:-}" ] || [ -n "${7:-}" ]; then
        echo "Error: optional mask syntax is --mask-config PATH" >&2
        exit 1
    fi
    MASK_CONFIG="$6"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/orbslam_mask_helpers.sh"

if [ -z "$SESSION_DIR" ] || [ -z "$ORB_SLAM_DIR" ]; then
    echo "Usage: $0 <session_dir> <orb_slam_dir> [skip_existing] [visualization] [--mask-config PATH]"
    echo "  session_dir:    path containing episode_* folders"
    echo "  orb_slam_dir:   path to ORB_SLAM3 installation"
    echo "  skip_existing:  \"true\" to skip episodes with CameraTrajectory.txt (default: true)"
    echo "  visualization:  \"true\" to enable GUI (default: false)"
    echo "  --mask-config:  optional gripper-mask JSON; omitted means no masking"
    exit 1
fi

if ! sroi_configure_mask "$MASK_CONFIG"; then
    exit 1
fi

STEREO_KITTI="$ORB_SLAM_DIR/Examples/Stereo/stereo_kitti"
VOCAB="$ORB_SLAM_DIR/Vocabulary/ORBvoc.txt"

if [ ! -x "$STEREO_KITTI" ]; then
    echo "Error: stereo_kitti not found or not executable: $STEREO_KITTI"
    exit 1
fi

if [ ! -f "$VOCAB" ]; then
    echo "Error: Vocabulary file not found: $VOCAB"
    exit 1
fi

SESSION_DIR="${SESSION_DIR%/}"
ORB_SLAM_DIR="${ORB_SLAM_DIR%/}"

echo "============================================"
echo "ORB_SLAM3 Batch Processor (D405)"
echo "============================================"
echo "Session dir:   $SESSION_DIR"
echo "ORB_SLAM3 dir: $ORB_SLAM_DIR"
echo "Skip existing: $SKIP_EXIST"
echo "Visualization: $VISUALIZATION"
echo "Mask config:   ${SROI_MASK_CONFIG:-none}"
echo ""

EPISODE_FOLDERS=()
for dir in "$SESSION_DIR"/episode_*/; do
    [ -d "$dir" ] || continue
    dir="${dir%/}"
    if (ls "$dir"/left_*.png 1>/dev/null 2>&1 || ls "$dir"/left_*.jpg 1>/dev/null 2>&1) && \
       (ls "$dir"/right_*.png 1>/dev/null 2>&1 || ls "$dir"/right_*.jpg 1>/dev/null 2>&1); then
        EPISODE_FOLDERS+=("$dir")
    fi
done

TOTAL=${#EPISODE_FOLDERS[@]}
echo "Found $TOTAL episode folder(s)"

if [ "$TOTAL" -eq 0 ]; then
    echo "No episode folders found with left/right images"
    exit 1
fi

SUCCESS=0
SKIP=0
ERROR=0

for folder in "${EPISODE_FOLDERS[@]}"; do
    name=$(basename "$folder")
    outfile="$folder/CameraTrajectory.txt"
    idx=$((SUCCESS + SKIP + ERROR + 1))

    echo ""
    echo "[$idx/$TOTAL] $name"

    if [ "$SKIP_EXIST" = "true" ] && [ -f "$outfile" ]; then
        echo "  Skipped (already exists)"
        SKIP=$((SKIP + 1))
        continue
    fi

    cfg="$folder/orb_slam_realsense_d405.yaml"
    if [ ! -f "$cfg" ]; then
        echo "  Warning: $cfg not found, using default"
        cfg="$ORB_SLAM_DIR/Examples/Stereo/RealSense_D405.yaml"
    fi

    if ! sroi_prepare_orb_input "$folder"; then
        echo "  Error: failed to prepare masked ORB input"
        ERROR=$((ERROR + 1))
        continue
    fi

    "$STEREO_KITTI" "$VOCAB" "$cfg" "$SROI_ORB_INPUT_DIR" "$VISUALIZATION"

    if sroi_collect_trajectory "$folder"; then
        echo "  Done"
        SUCCESS=$((SUCCESS + 1))
    else
        echo "  Error: no CameraTrajectory.txt generated"
        ERROR=$((ERROR + 1))
    fi
    sroi_cleanup_mask_input
done

echo ""
echo "============================================"
echo "Summary: $TOTAL total, $SUCCESS ok, $SKIP skipped, $ERROR errors"
echo "============================================"
