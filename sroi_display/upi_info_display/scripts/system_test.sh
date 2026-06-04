#!/bin/bash
sudo docker container run -it --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    ros:noetic-desktop-full  \
    bash