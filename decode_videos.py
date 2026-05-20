#!/usr/bin/env python3
"""
Decode MP4 episode folders back to JPEG image sequences.

Recovers frames from left.mp4, right.mp4, color.mp4 in episode directories
that were encoded by record_realsense.py --encode-video. Produces JPEG images
(quality 90) and validates frame counts against timestamps.json.

Output directories replace the -mp4 postfix with -jpeg:
    episode_001-mp4/  ->  episode_001-jpeg/
    (within the same parent session directory)

Metadata files (camera info, times.txt, timestamps.json, ORB-SLAM yaml)
are copied alongside the decoded images.

Usage:
    # Decode a single episode
    python decode_videos.py /path/to/session-mp4/episode_001-mp4

    # Recursively decode all *-mp4 episode dirs under a session
    python decode_videos.py /path/to/session-mp4 --recursive

Dependencies:
    pip install av opencv-python
"""

import argparse
import json
import shutil
from pathlib import Path

import cv2

try:
    import av
except ImportError:
    print("PyAV is required: pip install av")
    raise SystemExit(1)


JPEG_QUALITY = 90
STREAM_NAMES = ["left", "right", "color"]


def decode_episode(episode_dir: Path, output_dir: Path):
    """Decode MP4 files in episode_dir to JPEG images in output_dir."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # Copy metadata files
    for name in ["camera_info_left.json", "camera_info_right.json",
                 "camera_info_color.json", "times.txt", "timestamps.json"]:
        src = episode_dir / name
        if src.exists():
            shutil.copy2(src, output_dir / name)

    # Copy ORB-SLAM yaml if present
    for yml in episode_dir.glob("*.yaml"):
        shutil.copy2(yml, output_dir / yml.name)

    # Load timestamps for validation
    ts_path = episode_dir / "timestamps.json"
    expected_frames = None
    if ts_path.exists():
        with open(ts_path) as f:
            ts_data = json.load(f)
            expected_frames = ts_data.get("frame_count")

    total_frames = None
    for stream_name in STREAM_NAMES:
        mp4_path = episode_dir / f"{stream_name}.mp4"
        if not mp4_path.exists():
            print(f"  {stream_name}.mp4 not found, skipping.")
            continue

        frame_count = 0
        with av.open(str(mp4_path)) as container:
            stream = container.streams.video[0]
            for frame in container.decode(stream):
                img = frame.to_ndarray(format="bgr24")
                out_path = output_dir / f"{stream_name}_{frame_count:06d}.jpg"
                cv2.imwrite(str(out_path), img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                frame_count += 1

        print(f"  {stream_name}: {frame_count} frames decoded")
        if total_frames is None:
            total_frames = frame_count

    if expected_frames is not None and total_frames != expected_frames:
        print(f"  WARNING: decoded {total_frames} frames, expected {expected_frames}")


def get_output_dir(input_dir: Path) -> Path:
    """Derive output dir by replacing -mp4 postfix with -jpeg."""
    name = input_dir.name
    if name.endswith("-mp4"):
        return input_dir.parent / (name[:-4] + "-jpeg")
    return input_dir.parent / (name + "-jpeg")


def find_mp4_dirs(path: Path):
    """Find all *-mp4 episode directories under path."""
    dirs = []
    for d in sorted(path.iterdir()):
        if d.is_dir() and d.name.endswith("-mp4") and (d / "timestamps.json").exists():
            dirs.append(d)
    return dirs


def main():
    parser = argparse.ArgumentParser(
        description="Decode MP4 episode folders back to JPEG images.",
    )
    parser.add_argument(
        "input",
        type=str,
        help="Path to an episode dir or session dir (with --recursive)",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        default=False,
        help="Recursively find and decode all *-mp4 episode dirs",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Path not found: {input_path}")
        return

    if args.recursive:
        dirs = find_mp4_dirs(input_path)
        if not dirs:
            print(f"No *-mp4 episode directories found under {input_path}")
            return
        print(f"Found {len(dirs)} episode(s) to decode")
        for d in dirs:
            out = get_output_dir(d)
            print(f"Decoding {d.name} -> {out.name}")
            decode_episode(d, out)
    else:
        out = get_output_dir(input_path)
        print(f"Decoding {input_path.name} -> {out.name}")
        decode_episode(input_path, out)

    print("Done.")


if __name__ == "__main__":
    main()
