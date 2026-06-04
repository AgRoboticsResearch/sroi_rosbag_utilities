#!/bin/bash
docker container run -it --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    --volume="/home/brl/rosbags":"/home/codes/rosbags":rw \
    ros:noetic-desktop-full \
    bash 