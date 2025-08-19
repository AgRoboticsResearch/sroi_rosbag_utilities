#!/usr/bin/env python3

"""
Example script showing how to use the SROI to LeRobot converter.
"""

import sys
import os
from pathlib import Path

# Add paths
sys.path.append('/home/zfei/codes/lerobot/src')
sys.path.append('/home/zfei/codes/sroi/sroi_rosbag_utilities/lerobot')

def main():
    """Example usage of the converter."""
    
    # Your data path
    data_path = "/mnt/ldata/data/spi/spi/rs435_2025-07-23-09-10-02_sb_lab_picking/postproc/rs435_2025-07-23-09-10-02_segment_1/"
    
    # Check if data exists
    if not os.path.exists(data_path):
        print(f"Error: Data path does not exist: {data_path}")
        return
    
    # Run the conversion
    from sroi_to_lerobot import main as convert_main
    
    # Simulate command line arguments
    sys.argv = [
        "sroi_to_lerobot.py",
        "--data_path", data_path,
        "--repo_id", "sroi/ee_manipulation_demo",
        "--fps", "30",
        "--root", "/tmp/lerobot_datasets",
        "--task", "Robot end-effector manipulation task from SROI data"
    ]
    
    print("Starting conversion...")
    print(f"Data path: {data_path}")
    print(f"Output dataset: sroi/ee_manipulation_demo")
    
    try:
        convert_main()
        print("Conversion completed successfully!")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
