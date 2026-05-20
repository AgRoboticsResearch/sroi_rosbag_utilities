#!/usr/bin/env python3
"""
Real-time AprilTag visualization using RealSense color stream.

Shows detected tag boundaries, IDs, and pixel distance between
left (ID 0) and right (ID 15) gripper tags. Useful for verifying
tag placement before running gripper_estimation_april_tag.py.

Controls:
    q - Quit
    r - Toggle ROI line display
    +/- - Adjust ROI height
"""

import argparse
import time

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("pyrealsense2 is required: pip install pyrealsense2")
    raise SystemExit(1)

try:
    from pupil_apriltags import Detector
except ImportError:
    print("pupil_apriltags is required: pip install pupil-apriltags")
    raise SystemExit(1)


CAMERA_DEFAULTS = {
    "realsense_d435i": {"width": 848, "height": 480, "fps": 30},
    "realsense_d405": {"width": 640, "height": 480, "fps": 30},
}


def draw_tag(img, tag, color):
    """Draw tag outline, center, and ID."""
    pts = tag.corners.astype(int)
    for i in range(4):
        cv2.line(img, tuple(pts[i]), tuple(pts[(i + 1) % 4]), color, 2)
    cx, cy = int(tag.center[0]), int(tag.center[1])
    cv2.circle(img, (cx, cy), 4, color, -1)
    cv2.putText(img, f"ID {tag.tag_id}", (cx - 20, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def main():
    parser = argparse.ArgumentParser(description="Real-time AprilTag visualization")
    parser.add_argument("--camera", default="realsense_d435i",
                        choices=list(CAMERA_DEFAULTS.keys()))
    parser.add_argument("--roi", type=int, default=390,
                        help="ROI cutoff from top (pixels, hide above this)")
    parser.add_argument("--tag-family", default="tag16h5")
    parser.add_argument("--left-id", type=int, default=0)
    parser.add_argument("--right-id", type=int, default=15)
    args = parser.parse_args()

    defaults = CAMERA_DEFAULTS[args.camera]

    # RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, defaults["width"], defaults["height"],
                         rs.format.bgr8, defaults["fps"])
    profile = pipeline.start(config)

    # AprilTag detector (same params as gripper_estimation_april_tag.py)
    at_detector = Detector(
        families=args.tag_family,
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    roi = args.roi
    show_roi = True
    fps_counter = 0
    fps_timer = time.time()
    display_fps = 0.0

    print(f"Camera: {args.camera} ({defaults['width']}x{defaults['height']})")
    print(f"Looking for tag16h5 IDs: {args.left_id} (left), {args.right_id} (right)")
    print("Controls: q=quit  r=toggle ROI  +/-=adjust ROI")

    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=10000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            display = img.copy()

            # ROI mask
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray[:roi, :] = 0

            # Detect
            results = at_detector.detect(gray)

            left_tag = None
            right_tag = None

            for tag in results:
                if tag.tag_id == args.left_id:
                    draw_tag(display, tag, (0, 255, 0))  # green
                    left_tag = tag
                elif tag.tag_id == args.right_id:
                    draw_tag(display, tag, (0, 255, 255))  # yellow
                    right_tag = tag
                else:
                    draw_tag(display, tag, (128, 128, 128))  # gray

            # Distance between tags
            info_lines = [f"Tags: {len(results)}  ROI: {roi}  FPS: {display_fps:.0f}"]
            if left_tag:
                diag = np.linalg.norm(left_tag.corners[0] - left_tag.corners[2])
                info_lines.append(f"Left (ID {args.left_id}): diag={diag:.1f}px")
            else:
                info_lines.append(f"Left (ID {args.left_id}): ---")

            if right_tag:
                diag = np.linalg.norm(right_tag.corners[0] - right_tag.corners[2])
                info_lines.append(f"Right (ID {args.right_id}): diag={diag:.1f}px")
            else:
                info_lines.append(f"Right (ID {args.right_id}): ---")

            if left_tag and right_tag:
                dist = abs(right_tag.center[0] - left_tag.center[0])
                info_lines.append(f"Pixel distance: {dist:.1f}px")
                # Draw connecting line
                cv2.line(display,
                         (int(left_tag.center[0]), int(left_tag.center[1])),
                         (int(right_tag.center[0]), int(right_tag.center[1])),
                         (0, 0, 255), 2)
                cv2.putText(display, f"{dist:.1f}px",
                            (int((left_tag.center[0] + right_tag.center[0]) / 2),
                             int((left_tag.center[1] + right_tag.center[1]) / 2) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Overlay info
            for i, line in enumerate(info_lines):
                cv2.putText(display, line, (10, 25 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(display, line, (10, 25 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            # ROI line
            if show_roi:
                cv2.line(display, (0, roi), (defaults["width"], roi), (0, 165, 255), 1)
                cv2.putText(display, f"ROI y={roi}", (10, roi - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

            cv2.imshow("AprilTag Visualization", display)

            # FPS
            fps_counter += 1
            elapsed = time.time() - fps_timer
            if elapsed >= 1.0:
                display_fps = fps_counter / elapsed
                fps_counter = 0
                fps_timer = time.time()

            # Key handling
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                show_roi = not show_roi
            elif key in (ord("+"), ord("=")):
                roi = min(roi + 10, defaults["height"])
            elif key == ord("-"):
                roi = max(roi - 10, 0)

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


if __name__ == "__main__":
    main()
