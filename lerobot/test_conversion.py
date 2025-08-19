#!/usr/bin/env python3

"""
Test script to convert SROI data to LeRobot dataset format.
"""

import sys
import os

# Change to lerobot directory and add to path
os.chdir('/home/zfei/codes/lerobot')
sys.path.insert(0, '/home/zfei/codes/lerobot/src')
sys.path.insert(0, '/home/zfei/codes/sroi/sroi_rosbag_utilities/lerobot')

from sroi_to_lerobot import create_lerobot_dataset

def test_conversion():
    """Test the conversion with the provided data."""
    
    data_path = "/mnt/ldata/data/spi/spi/rs435_2025-07-23-09-10-02_sb_lab_picking/postproc/rs435_2025-07-23-09-10-02_segment_1/"
    repo_id = "sroi/test_ee_manipulation"
    
    print(f"Converting data from: {data_path}")
    print(f"Creating dataset: {repo_id}")
    
    try:
        dataset = create_lerobot_dataset(
            data_path=data_path,
            repo_id=repo_id,
            fps=30,
            root="/tmp/lerobot_datasets",  # Store locally for testing
            push_to_hub=False,  # Don't push to hub during testing
            single_task="Robot end-effector manipulation from SROI data"
        )
        
        print("Conversion successful!")
        print(f"Dataset created with {dataset.num_frames} frames")
        print(f"Dataset episodes: {dataset.num_episodes}")
        
        return dataset
        
    except Exception as e:
        print(f"Error during conversion: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    test_conversion()
