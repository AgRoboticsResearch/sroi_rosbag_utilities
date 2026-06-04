#!/bin/bash
sudo docker container run --name upi_system -i --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    ros:noetic-spi-oak \
    bash -c "
        roslaunch /home/codes/raspberry_pi/upi_python/upi_info_display/scripts_oak/oak_d_sr.launch
    "&
