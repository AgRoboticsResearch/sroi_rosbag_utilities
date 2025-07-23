import os
import subprocess
import glob
import argparse

def main():

    # Set up argument parser
    parser = argparse.ArgumentParser(description="Run evo_ape and evo_rpe on matching folders.")
    parser.add_argument("--folder_pattern", type=str, required=True, 
                        help="Pattern to match folders (e.g., '/ldata/data/temp/spi_postproc/z1_rs_calib_lab*')")
    parser.add_argument("--save_path", type=str, required=True, 
                        help="Directory to save evo_ape and evo_rpe results")
    
    args = parser.parse_args()

    # Create the evo_results directory if it doesn't exist
    evo_results_dir = args.save_path
    if not os.path.exists(evo_results_dir):
        os.makedirs(evo_results_dir)


    # Get all folders matching the pattern
    folders = glob.glob(args.folder_pattern)
    print("Folders found:", folders)
    if not folders:
        print(f"No folders found matching pattern: {args.folder_pattern}")
        return

    # Iterate through each folder
    for folder in folders:
        folder_basename = os.path.basename(folder)
        print(f"Processing folder: {folder_basename}")

        # Change to the folder
        os.chdir(folder)

        # Run evo_ape command
        ape_output_zip = os.path.join(evo_results_dir, f"ape_{folder_basename}.zip")
        evo_ape_command = [
            "evo_ape", "kitti", "Arm_traj.txt", "SLAM_traj.txt", "-va", "--save_results", ape_output_zip
        ]
        print(f"Running: {' '.join(evo_ape_command)}")
        subprocess.run(evo_ape_command)

        # Run evo_rpe command
        rpe_output_zip = os.path.join(evo_results_dir, f"rpe_{folder_basename}.zip")
        evo_rpe_command = [
            "evo_rpe", "kitti", "Arm_traj.txt", "SLAM_traj.txt", "-va", "--save_results", rpe_output_zip
        ]
        print(f"Running: {' '.join(evo_rpe_command)}")
        subprocess.run(evo_rpe_command)

        # Change back to the parent directory
        os.chdir("..")

    print("All folders processed. Results saved in the 'evo_results' directory.")

if __name__ == "__main__":
    main()