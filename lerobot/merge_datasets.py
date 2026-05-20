#!/usr/bin/env python3
"""
Merge multiple LeRobot datasets into a single dataset.

Usage:
    python merge_datasets.py --datasets ds1 ds2 ds3 --output merged_dataset
    python merge_datasets.py --datasets sroi_lab_picking_260316 sroi_lab_picking_260317 \
        --output sroi_lab_picking_merged \
        --root /path/to/datasets
"""

import argparse
import logging
from pathlib import Path

from lerobot.datasets.dataset_tools import merge_datasets
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def check_compatibility(datasets: list[LeRobotDataset]) -> bool:
    """Check if all datasets are compatible for merging.

    Args:
        datasets: List of LeRobotDataset objects to check.

    Returns:
        True if all datasets are compatible, raises ValueError otherwise.
    """
    if len(datasets) < 2:
        raise ValueError("Need at least 2 datasets to merge")

    ref = datasets[0]
    ref_fps = ref.meta.fps
    ref_robot_type = ref.meta.robot_type
    ref_features = ref.meta.features

    for i, ds in enumerate(datasets[1:], start=1):
        if ds.meta.fps != ref_fps:
            raise ValueError(
                f"Dataset {i} has fps={ds.meta.fps}, expected {ref_fps}"
            )
        if ds.meta.robot_type != ref_robot_type:
            raise ValueError(
                f"Dataset {i} has robot_type={ds.meta.robot_type}, expected {ref_robot_type}"
            )
        if ds.meta.features != ref_features:
            raise ValueError(
                f"Dataset {i} has different features than the first dataset"
            )

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Merge multiple LeRobot datasets into a single dataset"
    )
    parser.add_argument(
        "--datasets",
        nargs="+",
        required=True,
        help="List of dataset repo_ids to merge",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output repo_id for the merged dataset",
    )
    parser.add_argument(
        "--root",
        type=str,
        default=None,
        help="Root directory containing all datasets (default: same directory as first dataset)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Output directory for merged dataset (default: same as root)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose logging",
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )

    if len(args.datasets) < 2:
        raise ValueError("Need at least 2 datasets to merge")

    # Determine root directory
    if args.root:
        root = Path(args.root)
    else:
        # Use the first dataset's parent directory as root
        # Try common locations
        possible_roots = [
            Path("/home/hls/codes/sroi_rosbag_utilities/sroi_lerobot_datasets"),
            Path.cwd(),
        ]
        root = None
        for r in possible_roots:
            if (r / args.datasets[0]).exists():
                root = r
                break
        if root is None:
            raise ValueError(
                f"Could not find dataset {args.datasets[0]}. "
                "Please specify --root directory."
            )

    output_dir = Path(args.output_dir) if args.output_dir else root / args.output

    # Load datasets
    logging.info(f"Loading {len(args.datasets)} datasets from {root}")
    datasets = []
    total_episodes = 0
    total_frames = 0

    for repo_id in args.datasets:
        ds_path = root / repo_id
        if not ds_path.exists():
            raise ValueError(f"Dataset not found: {ds_path}")

        ds = LeRobotDataset(repo_id=repo_id, root=root)
        datasets.append(ds)
        total_episodes += ds.meta.total_episodes
        total_frames += ds.meta.total_frames
        logging.info(
            f"  {repo_id}: {ds.meta.total_episodes} episodes, {ds.meta.total_frames} frames"
        )

    # Check compatibility
    logging.info("Checking dataset compatibility...")
    check_compatibility(datasets)
    logging.info("All datasets are compatible")

    # Merge datasets
    logging.info(f"Merging datasets into {args.output}...")
    merged = merge_datasets(
        datasets=datasets,
        output_repo_id=args.output,
        output_dir=output_dir,
    )

    # Summary
    logging.info("=" * 50)
    logging.info("Merge complete!")
    logging.info(f"Output: {output_dir}")
    logging.info(f"Total episodes: {merged.meta.total_episodes} (expected: {total_episodes})")
    logging.info(f"Total frames: {merged.meta.total_frames} (expected: {total_frames})")
    logging.info("=" * 50)


if __name__ == "__main__":
    main()
