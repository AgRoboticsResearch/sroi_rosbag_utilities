#!/bin/bash
docker container run -i --rm --privileged --net=host \
    --volume=/dev:/dev \
    --volume="/home/brl/codes":"/home/codes":rw  \
    ros:noetic-spi-oak \
    bash -c "
    rosnode kill /upi_record
    "&