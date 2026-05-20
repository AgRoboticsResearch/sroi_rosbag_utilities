#!/usr/bin/env python3
"""
RealSense Stereo IR + Color Recorder

Records synchronized stereo infrared and color frames from a RealSense camera
using pyrealsense2. Produces output directories directly compatible with the
ORB-SLAM pipeline (orbslam_batch.sh, transform_trajectory.py, sroi_to_lerobot.py).

Controls (in the OpenCV window):
    r - Start recording a new episode
    s - Stop recording current episode
    q - Quit

Usage:
    python record_realsense.py --output /path/to/output --camera realsense_d435i
    python record_realsense.py -o /path/to/output --camera realsense_d405
"""

import argparse
import json
import select
import shutil
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("pyrealsense2 is required: pip install pyrealsense2")
    raise SystemExit(1)


# ---------------------------------------------------------------------------
# Camera defaults (IR resolution, color resolution, FPS)
# ---------------------------------------------------------------------------
CAMERA_DEFAULTS = {
    "realsense_d435i": {
        "ir_width": 848,
        "ir_height": 480,
        "ir_fps": 30,
        "color_width": 848,
        "color_height": 480,
        "color_fps": 30,
    },
    "realsense_d405": {
        "ir_width": 640,
        "ir_height": 480,
        "ir_fps": 30,
        "color_width": 640,
        "color_height": 480,
        "color_fps": 30,
    },
}


# ---------------------------------------------------------------------------
# Camera info helpers
# ---------------------------------------------------------------------------
def get_intrinsics(pipeline_profile, stream_type, stream_index=0):
    """Get intrinsics for a specific stream."""
    for profile in pipeline_profile.get_streams():
        if (
            profile.stream_type() == stream_type
            and profile.stream_index() == stream_index
        ):
            return profile.as_video_stream_profile().get_intrinsics()
    return None


def get_extrinsics(pipeline_profile, from_stream, from_idx, to_stream, to_idx):
    """Get extrinsics (rotation + translation) between two streams."""
    from_profile = None
    to_profile = None
    for profile in pipeline_profile.get_streams():
        if (
            profile.stream_type() == from_stream
            and profile.stream_index() == from_idx
        ):
            from_profile = profile
        if (
            profile.stream_type() == to_stream
            and profile.stream_index() == to_idx
        ):
            to_profile = profile
    if from_profile and to_profile:
        return from_profile.get_extrinsics_to(to_profile)
    return None


def make_camera_info(intrinsics, width, height, baseline_tx=0.0):
    """Convert pyrealsense2 intrinsics to ROS camera_info JSON format.

    This matches the format produced by extract_stereo_rosbags.py and
    consumed by create_orb_slam_yaml.py.
    """
    return {
        "height": height,
        "width": width,
        "distortion_model": "plumb_bob",
        "D": list(intrinsics.coeffs[:5]) if hasattr(intrinsics, "coeffs") else [0.0] * 5,
        "K": [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0,
        ],
        "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "P": [
            intrinsics.fx, 0.0, intrinsics.ppx, baseline_tx,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ],
    }


# ---------------------------------------------------------------------------
# Recording session
# ---------------------------------------------------------------------------
class RecordingSession:
    """Manages episode recording state and file I/O."""

    def __init__(self, output_dir: Path, camera_type: str = "realsense_d435i"):
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.camera_type = camera_type
        self.episode_count = 0
        self.is_recording = False
        self.frame_counter = 0
        self.episode_dir = None
        self.times_file = None

    def start_episode(self, camera_info_left, camera_info_right, camera_info_color):
        """Start recording a new episode."""
        if self.is_recording:
            print("Already recording. Press 's' to stop first.")
            return

        self.episode_count += 1
        self.frame_counter = 0
        self.episode_dir = self.output_dir / f"episode_{self.episode_count:03d}"
        self.episode_dir.mkdir(parents=True, exist_ok=True)

        # Save camera info JSONs
        for info, name in [
            (camera_info_left, "camera_info_left.json"),
            (camera_info_right, "camera_info_right.json"),
            (camera_info_color, "camera_info_color.json"),
        ]:
            with open(self.episode_dir / name, "w") as f:
                json.dump(info, f, indent=2)

        # D405 has fixed calibration — copy the ORB-SLAM YAML directly
        if self.camera_type == "realsense_d405":
            template = Path(__file__).parent / "orb_slam_yaml" / "RealSense_D405.yaml"
            if template.exists():
                shutil.copy2(template, self.episode_dir / "orb_slam_realsense_d405.yaml")

        self.times_file = open(self.episode_dir / "times.txt", "w")
        self.is_recording = True
        print(f"Recording episode {self.episode_count} -> {self.episode_dir}")

    def save_frame(self, timestamp, left_img, right_img, color_img):
        """Save a single frame (left IR, right IR, color) and timestamp."""
        if not self.is_recording:
            return

        idx = self.frame_counter
        cv2.imwrite(str(self.episode_dir / f"left_{idx:06d}.png"), left_img)
        cv2.imwrite(str(self.episode_dir / f"right_{idx:06d}.png"), right_img)
        cv2.imwrite(str(self.episode_dir / f"color_{idx:06d}.png"), color_img)
        self.times_file.write(f"{timestamp:.6f}\n")
        self.frame_counter += 1

    def stop_episode(self):
        """Stop recording the current episode."""
        if not self.is_recording:
            print("Not currently recording.")
            return

        self.times_file.close()
        self.is_recording = False
        print(
            f"Stopped episode {self.episode_count}: "
            f"{self.frame_counter} frames saved to {self.episode_dir}"
        )


# ---------------------------------------------------------------------------
# OpenCV preview helpers
# ---------------------------------------------------------------------------
def build_preview(color_img, left_img, right_img, session, fps):
    """Build the composite preview image with status overlay."""
    h, w = color_img.shape[:2]

    # IR images are grayscale — resize to half height for side panel
    small_h = h // 2
    left_small = cv2.resize(left_img, (w // 2, small_h))
    right_small = cv2.resize(right_img, (w // 2, small_h))

    # Convert grayscale IR to BGR so dimensions match color image
    if len(left_small.shape) == 2:
        left_small = cv2.cvtColor(left_small, cv2.COLOR_GRAY2BGR)
    if len(right_small.shape) == 2:
        right_small = cv2.cvtColor(right_small, cv2.COLOR_GRAY2BGR)

    # Stack IR images vertically
    ir_panel = np.vstack([left_small, right_small])

    # Stack color + IR side by side
    preview = np.hstack([color_img, ir_panel])

    # Status overlay
    if session.is_recording:
        status = f"RECORDING episode {session.episode_count}"
        color = (0, 0, 255)  # red
    else:
        status = "IDLE"
        color = (0, 200, 0)  # green

    cv2.putText(preview, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(
        preview,
        f"FPS: {fps:.0f}",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (200, 200, 200),
        1,
    )
    cv2.putText(
        preview,
        f"Episodes: {session.episode_count}",
        (10, 85),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (200, 200, 200),
        1,
    )

    if session.is_recording:
        cv2.putText(
            preview,
            f"Frame: {session.frame_counter}",
            (10, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            1,
        )

    # Controls help
    cv2.putText(
        preview,
        "[r] record  [s] stop  [q] quit",
        (10, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (180, 180, 180),
        1,
    )

    return preview


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Record stereo IR + color from RealSense camera for ORB-SLAM pipeline",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="./recordings",
        help="Output root directory (default: ./recordings)",
    )
    parser.add_argument(
        "--camera",
        type=str,
        default="realsense_d435i",
        choices=list(CAMERA_DEFAULTS.keys()),
        help="Camera type (default: realsense_d435i)",
    )
    args = parser.parse_args()

    # Create session directory with unix timestamp
    session_timestamp = int(time.time())
    session_dir = Path(args.output) / str(session_timestamp)
    session = RecordingSession(session_dir, camera_type=args.camera)

    # Setup RealSense pipeline
    defaults = CAMERA_DEFAULTS[args.camera]
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.infrared, 1,
        defaults["ir_width"], defaults["ir_height"],
        rs.format.y8, defaults["ir_fps"],
    )
    config.enable_stream(
        rs.stream.infrared, 2,
        defaults["ir_width"], defaults["ir_height"],
        rs.format.y8, defaults["ir_fps"],
    )
    config.enable_stream(
        rs.stream.color,
        defaults["color_width"], defaults["color_height"],
        rs.format.bgr8, defaults["color_fps"],
    )

    print(f"Starting RealSense pipeline ({args.camera})...")
    print(f"IR:  {defaults['ir_width']}x{defaults['ir_height']} @ {defaults['ir_fps']}fps")
    print(f"Color: {defaults['color_width']}x{defaults['color_height']} @ {defaults['color_fps']}fps")
    print(f"Output: {session_dir}")

    pipeline_profile = pipeline.start(config)

    # Disable IR emitter (removes laser dot pattern from IR images)
    device = pipeline_profile.get_device()
    for sensor in device.query_sensors():
        if sensor.supports(rs.option.emitter_enabled):
            sensor.set_option(rs.option.emitter_enabled, 0)
            print("IR emitter disabled (no laser dots in IR images)")
            break

    # Get camera intrinsics and extrinsics
    ir_left_intrinsics = get_intrinsics(pipeline_profile, rs.stream.infrared, 1)
    ir_right_intrinsics = get_intrinsics(pipeline_profile, rs.stream.infrared, 2)
    color_intrinsics = get_intrinsics(pipeline_profile, rs.stream.color, 0)

    # Get baseline from extrinsics (translation in meters)
    extrinsics = get_extrinsics(
        pipeline_profile,
        rs.stream.infrared, 2,
        rs.stream.infrared, 1,
    )
    baseline_m = extrinsics.translation[0] if extrinsics else 0.0

    # Build camera info dicts (ROS camera_info format)
    camera_info_left = make_camera_info(ir_left_intrinsics, defaults["ir_width"], defaults["ir_height"])
    camera_info_right = make_camera_info(
        ir_right_intrinsics, defaults["ir_width"], defaults["ir_height"],
        baseline_tx=-ir_right_intrinsics.fx * baseline_m,
    )
    camera_info_color = make_camera_info(color_intrinsics, defaults["color_width"], defaults["color_height"])

    print(f"Baseline: {baseline_m * 1000:.2f} mm")
    print(f"IR Left fx={ir_left_intrinsics.fx:.2f} fy={ir_left_intrinsics.fy:.2f}")
    print()
    print("Controls: [r] record  [s] stop  [q] quit")

    # Warm up — discard first few frames
    print("Warming up camera...")
    for i in range(30):
        try:
            pipeline.wait_for_frames(timeout_ms=10000)
        except RuntimeError:
            if i < 5:
                print(f"  Frame {i}: timeout, retrying...")
                continue
            print("Camera not responding during warm-up. Check USB connection.")
            pipeline.stop()
            return

    # Main loop
    fps_counter = 0
    fps_timer = time.time()
    display_fps = 0.0

    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=10000)
            except RuntimeError:
                print("Camera frame timeout — camera may have disconnected.")
                break

            ir_left_frame = frames.get_infrared_frame(1)
            ir_right_frame = frames.get_infrared_frame(2)
            color_frame = frames.get_color_frame()

            if not ir_left_frame or not ir_right_frame or not color_frame:
                continue

            ir_left_img = np.asanyarray(ir_left_frame.get_data())
            ir_right_img = np.asanyarray(ir_right_frame.get_data())
            color_img = np.asanyarray(color_frame.get_data())

            # Record if active
            if session.is_recording:
                timestamp = frames.get_timestamp() / 1000.0  # ms -> seconds
                session.save_frame(timestamp, ir_left_img, ir_right_img, color_img)

            # FPS calculation
            fps_counter += 1
            elapsed = time.time() - fps_timer
            if elapsed >= 1.0:
                display_fps = fps_counter / elapsed
                fps_counter = 0
                fps_timer = time.time()

            # Build and show preview
            preview = build_preview(color_img, ir_left_img, ir_right_img, session, display_fps)
            cv2.imshow("RealSense Recorder", preview)

            # Key handling
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                session.start_episode(camera_info_left, camera_info_right, camera_info_color)
            elif key == ord("s"):
                session.stop_episode()
            elif key == ord("q"):
                if session.is_recording:
                    session.stop_episode()
                break

    except KeyboardInterrupt:
        print("\nInterrupted.")
        if session.is_recording:
            session.stop_episode()
    finally:
        pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        print(f"\nSession saved to: {session_dir}")
        print(f"Total episodes: {session.episode_count}")


if __name__ == "__main__":
    main()
