import os
import re

# Get list of all zip files in the current directory
files = [f for f in os.listdir() if f.endswith('.zip')]

# Sort files by timestamp (chronological order)
files.sort(key=lambda x: re.search(r'(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})', x).group())

# Rename files sequentially
for idx, filename in enumerate(files, start=1):
    new_name = f"run{idx}.zip"
    os.rename(filename, new_name)
    print(f"Renamed {filename} ➔ {new_name}")