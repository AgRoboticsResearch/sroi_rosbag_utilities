#!/bin/bash
# Shared optional gripper-mask support for the ORB-SLAM batch scripts.

SROI_MASK_HELPER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SROI_MASK_TOOL="$SROI_MASK_HELPER_DIR/../apply_gripper_mask.py"
SROI_MASK_CONFIG=""
SROI_MASK_TEMP_DIR=""
SROI_ORB_INPUT_DIR=""

sroi_configure_mask() {
    local config="${1:-}"
    if [ -z "$config" ]; then
        SROI_MASK_CONFIG=""
        return 0
    fi
    if [ ! -f "$config" ]; then
        echo "Error: mask config not found: $config" >&2
        return 1
    fi
    if [ ! -f "$SROI_MASK_TOOL" ]; then
        echo "Error: mask utility not found: $SROI_MASK_TOOL" >&2
        return 1
    fi
    SROI_MASK_CONFIG="$(cd "$(dirname "$config")" && pwd)/$(basename "$config")"
}

sroi_cleanup_mask_input() {
    if [ -n "$SROI_MASK_TEMP_DIR" ] && [ -d "$SROI_MASK_TEMP_DIR" ]; then
        rm -rf -- "$SROI_MASK_TEMP_DIR"
    fi
    SROI_MASK_TEMP_DIR=""
    SROI_ORB_INPUT_DIR=""
}

sroi_prepare_orb_input() {
    local source_episode="$1"
    sroi_cleanup_mask_input
    if [ -z "$SROI_MASK_CONFIG" ]; then
        SROI_ORB_INPUT_DIR="$source_episode"
        return 0
    fi

    SROI_MASK_TEMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/sroi-orb-mask.XXXXXX")" || return 1
    SROI_ORB_INPUT_DIR="$SROI_MASK_TEMP_DIR/episode"
    if ! python3 "$SROI_MASK_TOOL" "$source_episode" "$SROI_ORB_INPUT_DIR" \
        --config "$SROI_MASK_CONFIG"; then
        sroi_cleanup_mask_input
        return 1
    fi
}

sroi_collect_trajectory() {
    local destination_episode="$1"
    if [ -z "$SROI_MASK_CONFIG" ]; then
        [ -f "$destination_episode/CameraTrajectory.txt" ]
        return
    fi
    local masked_trajectory="$SROI_ORB_INPUT_DIR/CameraTrajectory.txt"
    if [ ! -f "$masked_trajectory" ]; then
        return 1
    fi
    local destination="$destination_episode/CameraTrajectory.txt"
    local temporary="$destination.tmp"
    if ! cp "$masked_trajectory" "$temporary"; then
        rm -f -- "$temporary"
        return 1
    fi
    mv "$temporary" "$destination"
}

trap sroi_cleanup_mask_input EXIT
trap 'sroi_cleanup_mask_input; exit 130' INT TERM
