#!/bin/bash
docker container run -i --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    ros:noetic-desktop-full \
    bash -c "
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
        /tf_static \
        /upi/status/is_action \
        __name:=upi_record
    "&