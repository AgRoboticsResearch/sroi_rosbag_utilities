#!/usr/bin/env python3
"""
SROI Data Processing Pipeline

A unified script to process ROS bag data through the complete pipeline:
1. Segment bags and extract images
2. Estimate gripper distances from AprilTags
3. Transform ORB-SLAM trajectories (run ORB-SLAM separately in Docker)
4. Visualize trajectories
5. Convert to LeRobot dataset

Usage:
    # Run full pipeline
    python run_pipeline.py -i input_bags/ -o output/ --camera realsense_d435i

    # Run specific steps
    python run_pipeline.py -i input_bags/ -o output/ --steps segment,gripper,transform

    # Skip completed steps
    python run_pipeline.py -i input_bags/ -o output/ --skip-existing

    # Dry run to see what would be executed
    python run_pipeline.py -i input_bags/ -o output/ --dry-run

Author: SROI ROS Bag Utilities
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import List, Optional


class PipelineRunner:
    """Runs the SROI data processing pipeline."""

    def __init__(
        self,
        input_dir: str,
        output_dir: str,
        camera: str = "realsense_d435i",
        compressed: bool = True,
        skip_existing: bool = False,
        dry_run: bool = False,
        verbose: bool = False,
    ):
        self.input_dir = Path(input_dir).resolve()
        self.output_dir = Path(output_dir).resolve()
        self.camera = camera
        self.compressed = compressed
        self.skip_existing = skip_existing
        self.dry_run = dry_run
        self.verbose = verbose
        self.script_dir = Path(__file__).parent.resolve()

    def log(self, msg: str, level: str = "INFO"):
        """Print log message."""
        prefix = f"[{level}]"
        if level == "STEP":
            prefix = "\n" + "=" * 60 + f"\n  STEP: {msg}\n" + "=" * 60
        print(f"{prefix} {msg}")

    def run_command(self, cmd: List[str], description: str = "") -> bool:
        """Run a shell command."""
        cmd_str = " ".join(str(c) for c in cmd)
        self.log(f"Running: {cmd_str}")

        if self.dry_run:
            self.log("DRY RUN - skipping execution")
            return True

        try:
            result = subprocess.run(
                cmd,
                cwd=self.script_dir,
                capture_output=not self.verbose,
                text=True,
            )
            if result.returncode != 0:
                self.log(f"Command failed with code {result.returncode}", "ERROR")
                if result.stderr:
                    self.log(result.stderr, "ERROR")
                return False
            return True
        except Exception as e:
            self.log(f"Exception: {e}", "ERROR")
            return False

    def step_segment_extract(self) -> bool:
        """Step 1: Segment bags and extract images."""
        self.log("Segment bags and extract images", "STEP")

        cmd = [
            sys.executable,
            str(self.script_dir / "rosbag_segment_extract.py"),
            "-i", str(self.input_dir),
            "-o", str(self.output_dir),
            self.camera,
        ]
        if self.compressed:
            cmd.append("--compressed")
        if self.skip_existing:
            cmd.append("--skip-existing")

        return self.run_command(cmd)

    def step_gripper_estimation(self) -> bool:
        """Step 2: Estimate gripper distances from AprilTags."""
        self.log("Estimate gripper distances", "STEP")

        cmd = [
            sys.executable,
            str(self.script_dir / "gripper_estimation_batch.py"),
            "-i", str(self.output_dir),
        ]
        if self.skip_existing:
            cmd.append("--skip-existing")

        return self.run_command(cmd)

    def step_orbslam(self, skip_existing: bool = True, visualization: bool = False) -> bool:
        """Step 3: Run ORB-SLAM3 (requires Docker).

        Note: This step cannot be run automatically. The user must:
        1. Enter Docker: ./run_docker.sh
        2. Run: ./orbslam_batch.sh <output_dir> <skip_existing> <visualization>
        """
        self.log("ORB-SLAM3 processing", "STEP")

        skip_str = "true" if skip_existing else "false"
        viz_str = "true" if visualization else "false"

        self.log("ORB-SLAM3 must be run manually inside Docker:")
        self.log("  1. Enter Docker: ./run_docker.sh")
        self.log(f"  2. Run: ./orbslam_batch.sh {self.output_dir} {skip_str} {viz_str}")

        # Check if ORB-SLAM has been run (look for CameraTrajectory.txt files)
        trajectory_files = list(self.output_dir.rglob("CameraTrajectory.txt"))
        if trajectory_files:
            self.log(f"Found {len(trajectory_files)} existing CameraTrajectory.txt files")
            return True
        else:
            self.log("No CameraTrajectory.txt files found. Run ORB-SLAM first.", "WARN")
            return False

    def step_transform_trajectory(self) -> bool:
        """Step 4: Transform trajectories to initial camera frame."""
        self.log("Transform trajectories", "STEP")

        cmd = [
            sys.executable,
            str(self.script_dir / "transform_trajectory.py"),
            str(self.output_dir),
            "--recursive",
        ]

        return self.run_command(cmd)

    def step_visualize(self, scale: str = "uniform") -> bool:
        """Step 5: Visualize trajectories."""
        self.log("Visualize trajectories", "STEP")

        cmd = [
            sys.executable,
            str(self.script_dir / "batch_visualize_trajectory.py"),
            str(self.output_dir),
            "--scale", scale,
        ]
        if self.skip_existing:
            cmd.append("--skip")

        return self.run_command(cmd)

    def step_lerobot(
        self,
        repo_id: str,
        fps: int = 30,
        root: Optional[str] = None,
        task: str = "manipulation task",
    ) -> bool:
        """Step 6: Convert to LeRobot dataset."""
        self.log("Convert to LeRobot dataset", "STEP")

        # LeRobot script must be run from lerobot directory
        lerobot_dir = Path("/home/hls/codes/lerobot")
        if not lerobot_dir.exists():
            self.log(f"LeRobot directory not found: {lerobot_dir}", "ERROR")
            return False

        cmd = [
            sys.executable,
            str(self.script_dir / "lerobot" / "sroi_to_lerobot.py"),
            "--data_path", str(self.output_dir),
            "--repo_id", repo_id,
            "--fps", str(fps),
            "--task", task,
        ]
        if root:
            cmd.extend(["--root", root])

        # Run from lerobot directory
        self.log(f"Running from {lerobot_dir}")
        cmd_str = " ".join(str(c) for c in cmd)
        self.log(f"Running: {cmd_str}")

        if self.dry_run:
            self.log("DRY RUN - skipping execution")
            return True

        try:
            result = subprocess.run(
                cmd,
                cwd=lerobot_dir,
                capture_output=not self.verbose,
                text=True,
            )
            if result.returncode != 0:
                self.log(f"Command failed with code {result.returncode}", "ERROR")
                if result.stderr:
                    self.log(result.stderr, "ERROR")
                return False
            return True
        except Exception as e:
            self.log(f"Exception: {e}", "ERROR")
            return False

    def run(
        self,
        steps: Optional[List[str]] = None,
        skip_orbslam: bool = True,
        lerobot_repo: Optional[str] = None,
        lerobot_task: str = "manipulation task",
        lerobot_root: Optional[str] = None,
    ):
        """Run the pipeline."""
        all_steps = ["segment", "gripper", "orbslam", "transform", "visualize", "lerobot"]
        if steps is None:
            steps = all_steps

        self.log(f"Input:  {self.input_dir}")
        self.log(f"Output: {self.output_dir}")
        self.log(f"Steps:  {', '.join(steps)}")
        self.log(f"Skip existing: {self.skip_existing}")
        self.log(f"Dry run: {self.dry_run}")

        results = {}

        # Step 1: Segment and extract
        if "segment" in steps:
            results["segment"] = self.step_segment_extract()
            if not results["segment"]:
                self.log("Segment step failed, stopping pipeline", "ERROR")
                return results

        # Step 2: Gripper estimation
        if "gripper" in steps:
            results["gripper"] = self.step_gripper_estimation()
            if not results["gripper"]:
                self.log("Gripper estimation failed, stopping pipeline", "ERROR")
                return results

        # Step 3: ORB-SLAM
        if "orbslam" in steps:
            if skip_orbslam:
                self.log("Skipping ORB-SLAM (run manually in Docker)")
                results["orbslam"] = True
            else:
                results["orbslam"] = self.step_orbslam(skip_existing=self.skip_existing)
                if not results["orbslam"]:
                    self.log("ORB-SLAM step incomplete. Run manually in Docker.", "WARN")

        # Step 4: Transform trajectory
        if "transform" in steps:
            results["transform"] = self.step_transform_trajectory()
            if not results["transform"]:
                self.log("Transform step failed", "ERROR")

        # Step 5: Visualize
        if "visualize" in steps:
            results["visualize"] = self.step_visualize()
            if not results["visualize"]:
                self.log("Visualization step failed", "WARN")

        # Step 6: LeRobot conversion
        if "lerobot" in steps:
            if lerobot_repo is None:
                self.log("Skipping LeRobot conversion (no --lerobot-repo specified)", "WARN")
                results["lerobot"] = False
            else:
                results["lerobot"] = self.step_lerobot(
                    repo_id=lerobot_repo,
                    task=lerobot_task,
                    root=lerobot_root,
                )

        # Summary
        self.log("Pipeline Results:", "STEP")
        for step, success in results.items():
            status = "✓" if success else "✗"
            self.log(f"  {status} {step}")

        return results


def main():
    parser = argparse.ArgumentParser(
        description="SROI Data Processing Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Run full pipeline
    python run_pipeline.py -i bags/ -o output/ --camera realsense_d435i

    # Run specific steps
    python run_pipeline.py -i bags/ -o output/ --steps segment,gripper,transform

    # Run with LeRobot conversion
    python run_pipeline.py -i bags/ -o output/ --lerobot-repo user/dataset --lerobot-task "pick object"

    # Dry run
    python run_pipeline.py -i bags/ -o output/ --dry-run

Available steps: segment, gripper, orbslam, transform, visualize, lerobot
        """,
    )

    parser.add_argument(
        "-i", "--input",
        required=True,
        help="Input directory containing ROS bag files",
    )
    parser.add_argument(
        "-o", "--output",
        required=True,
        help="Output directory for processed data",
    )
    parser.add_argument(
        "--camera",
        default="realsense_d435i",
        choices=["realsense_d435i", "oak"],
        help="Camera type (default: realsense_d435i)",
    )
    parser.add_argument(
        "--steps",
        help="Comma-separated list of steps to run (default: all)",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip processing if output already exists",
    )
    parser.add_argument(
        "--compressed",
        action="store_true",
        default=True,
        help="Use compressed images (default: True)",
    )
    parser.add_argument(
        "--skip-orbslam",
        action="store_true",
        default=True,
        help="Skip ORB-SLAM step (run manually in Docker)",
    )
    parser.add_argument(
        "--lerobot-repo",
        help="LeRobot repository ID (e.g., 'user/dataset')",
    )
    parser.add_argument(
        "--lerobot-task",
        default="manipulation task",
        help="Task description for LeRobot dataset",
    )
    parser.add_argument(
        "--lerobot-root",
        help="Root directory for LeRobot dataset storage",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show commands without executing",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show detailed output",
    )

    args = parser.parse_args()

    # Parse steps
    steps = None
    if args.steps:
        steps = [s.strip().lower() for s in args.steps.split(",")]

    # Run pipeline
    runner = PipelineRunner(
        input_dir=args.input,
        output_dir=args.output,
        camera=args.camera,
        compressed=args.compressed,
        skip_existing=args.skip_existing,
        dry_run=args.dry_run,
        verbose=args.verbose,
    )

    results = runner.run(
        steps=steps,
        skip_orbslam=args.skip_orbslam,
        lerobot_repo=args.lerobot_repo,
        lerobot_task=args.lerobot_task,
        lerobot_root=args.lerobot_root,
    )

    # Exit with error if any step failed
    if not all(results.values()):
        sys.exit(1)


if __name__ == "__main__":
    main()
