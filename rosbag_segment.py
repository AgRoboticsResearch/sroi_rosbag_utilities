import rosbag
from rosbag import Bag
import argparse
import os
import glob

def extract_true_segments(input_file, output_template=None):
    # 读取所有状态消息
    status_data = []
    with Bag(input_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/upi/status/is_action']):
            status_data.append((t.to_sec(), msg.data))

    # 检测连续true片段
    segments = []
    current_segment = None
    for timestamp, value in status_data:
        if value:  # 当前为true
            if not current_segment:  # 新片段开始
                current_segment = {
                    'start': timestamp,
                    'end': timestamp,
                    'start_msg_time': None,
                    'end_msg_time': None
                }
            else:  # 延续当前片段
                current_segment['end'] = timestamp
        else:  # 当前为false
            if current_segment:  # 结束当前片段
                segments.append(current_segment)
                current_segment = None

    # 处理最后一个片段
    if current_segment:
        segments.append(current_segment)

    # 分割原始bag文件
    for i, seg in enumerate(segments):
        if output_template is None:
            base_name = os.path.splitext(os.path.basename(input_file))[0]
            output_file = f"{base_name}_segment_{i+1}.bag"
        else:
            output_file = output_template.format(i+1)
        
        print(f"Processing segment {i+1}: {seg['start']} to {seg['end']}")

        with Bag(input_file, 'r') as inbag:
            with Bag(output_file, 'w') as outbag:
                # 遍历所有消息
                for topic, msg, t in inbag.read_messages():
                    msg_time = t.to_sec()
                    if seg['start'] <= msg_time <= seg['end']:
                        outbag.write(topic, msg, t)

    return len(segments)

def process_bag_file(input_file, output_dir=None, template=None):
    """Process a single bag file and return the number of segments."""
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        if template:
            output_template = os.path.join(output_dir, template)
        else:
            base_name = os.path.splitext(os.path.basename(input_file))[0]
            output_template = os.path.join(output_dir, f"{base_name}_segment_{{}}.bag")
    else:
        output_template = template

    print(f"\nProcessing file: {input_file}")
    num_segments = extract_true_segments(input_file, output_template)
    print(f"Found {num_segments} segments in {os.path.basename(input_file)}")
    return num_segments

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract true segments from ROS bag files.')
    parser.add_argument('input', type=str, help='Input bag file or directory containing bag files')
    parser.add_argument('-o', '--output', type=str, help='Output directory for segmented bag files')
    parser.add_argument('-t', '--template', type=str, help='Output filename template (e.g., "segment_{}.bag")')
    parser.add_argument('-p', '--pattern', type=str, default='*.bag', help='Pattern to match bag files (default: *.bag)')
    args = parser.parse_args()

    # Determine if input is a file or directory
    if os.path.isfile(args.input):
        # Single file processing
        process_bag_file(args.input, args.output, args.template)
    else:
        # Directory processing
        if not os.path.isdir(args.input):
            print(f"Error: Input path '{args.input}' does not exist")
            exit(1)

        # Find all bag files in the directory
        search_pattern = os.path.join(args.input, args.pattern)
        bag_files = sorted(glob.glob(search_pattern))
        
        if not bag_files:
            print(f"No bag files found matching pattern '{args.pattern}' in {args.input}")
            exit(1)

        print(f"Found {len(bag_files)} bag files to process")
        
        # Process each file
        total_segments = 0
        for bag_file in bag_files:
            total_segments += process_bag_file(bag_file, args.output, args.template)
        
        print(f"\nProcessing complete. Total segments found: {total_segments}")