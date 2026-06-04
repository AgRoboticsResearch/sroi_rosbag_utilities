#!/bin/bash
docker container run -i --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    ros:noetic-spi-oak \
    bash -c "
        echo 'Starting Action Server...'
        python3 /home/codes/raspberry_pi/upi_python/upi_info_display/action_node.py &
        sleep 1
        echo 'Starting Topic Monitor OAK...'
        python3 /home/codes/raspberry_pi/upi_python/upi_info_display/topic_monitor_oak.py
    "&


