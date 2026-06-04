#!/bin/bash

docker container run -i --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    --volume="/home/brl/rosbags":"/home/codes/rosbags":rw \
    ros:noetic-desktop-full \
    bash -c "
        echo 'Starting ROS launch...'
        roslaunch /home/codes/d435i.launch &
        sleep 10  # 等待 10 秒，以确保节点启动

        echo 'Starting rosbag recording...'
        rosbag record -o /home/codes/rosbags/rs435 \
        /camera/imu \
        /camera/accel/imu_info \
        /camera/accel/metadata \
        /camera/accel/sample \
        /camera/color/camera_info \
        /camera/color/image_raw/compressed \
        /camera/extrinsics/depth_to_color \
        /camera/extrinsics/depth_to_infra1 \
        /camera/extrinsics/depth_to_infra2 \
        /camera/gyro/imu_info \
        /camera/gyro/metadata \
        /camera/gyro/sample \
        /camera/infra1/camera_info \
        /camera/infra1/image_rect_raw/compressed \
        /camera/infra2/camera_info \
        /camera/infra2/image_rect_raw/compressed \
        /tf \
        /tf_static &

        echo 'Container is running, blocking process to keep container alive...'
        tail -f /dev/null  # 保持容器运行
    "&