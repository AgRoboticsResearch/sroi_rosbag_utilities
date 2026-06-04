#!/bin/bash
docker container run -i --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    ros:noetic-spi-oak \
    bash -c "
        rosbag record -o /home/codes/rosbags/oak_d_sr \
        /oak/imu/data \
        /oak/left/image_raw/compressed \
        /oak/right/image_raw/compressed \
        /upi/status/is_action \
        __name:=upi_record
    "&