#!/bin/bash
docker container run --name upi_system -i --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    ros:noetic-desktop-full \
    bash -c "
        echo 'Starting ROS launch...'
        roslaunch /home/codes/raspberry_pi/upi_python/upi_info_display/scripts/d435i.launch
    "&