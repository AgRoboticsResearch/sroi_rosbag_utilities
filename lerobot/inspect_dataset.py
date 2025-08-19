#!/usr/bin/env python3

"""
Script to inspect the converted LeRobot dataset.
"""

import sys
import os
import numpy as np

# Change to lerobot directory
os.chdir('/home/zfei/codes/lerobot')
sys.path.insert(0, '/home/zfei/codes/lerobot/src')

from lerobot.datasets.lerobot_dataset import LeRobotDataset

def inspect_dataset():
    """Load and inspect the converted dataset."""
    
    dataset_path = "/tmp/lerobot_datasets/sroi/test_ee_manipulation"
    
    print(f"Loading dataset from: {dataset_path}")
    
    try:
        # Load the dataset
        dataset = LeRobotDataset(repo_id="sroi/test_ee_manipulation", root="/tmp/lerobot_datasets")
        
        print(f"Dataset loaded successfully!")
        print(f"Number of episodes: {dataset.num_episodes}")
        print(f"Number of frames: {dataset.num_frames}")
        print(f"Features: {list(dataset.features.keys())}")
        print(f"FPS: {dataset.fps}")
        
        # Get a sample frame
        sample_frame = dataset[0]
        
        print("\nSample frame structure:")
        for key, value in sample_frame.items():
            if hasattr(value, 'shape'):
                print(f"  {key}: shape={value.shape}, dtype={value.dtype}")
            else:
                print(f"  {key}: {type(value)} = {value}")
        
        # Check action ranges
        actions = [dataset[i]['action'] for i in range(min(10, len(dataset)))]
        actions = np.array(actions)
        
        print(f"\nAction statistics (first 10 frames):")
        print(f"  Action shape: {actions.shape}")
        print(f"  Action mean: {np.mean(actions, axis=0)}")
        print(f"  Action std: {np.std(actions, axis=0)}")
        print(f"  Action min: {np.min(actions, axis=0)}")
        print(f"  Action max: {np.max(actions, axis=0)}")
        
        # Check observation state ranges
        states = [dataset[i]['observation.state'] for i in range(min(10, len(dataset)))]
        states = np.array(states)
        
        print(f"\nState statistics (first 10 frames):")
        print(f"  State shape: {states.shape}")
        print(f"  State mean: {np.mean(states, axis=0)}")
        print(f"  State std: {np.std(states, axis=0)}")
        print(f"  State min: {np.min(states, axis=0)}")
        print(f"  State max: {np.max(states, axis=0)}")
        
        # Check image
        image = sample_frame['observation.images.camera']
        print(f"\nImage statistics:")
        print(f"  Image shape: {image.shape}")
        print(f"  Image dtype: {image.dtype}")
        print(f"  Image min: {image.min().item()}")
        print(f"  Image max: {image.max().item()}")
        
        return dataset
        
    except Exception as e:
        print(f"Error loading dataset: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    inspect_dataset()
