#!/bin/bash

scriptDir=$(dirname $0 | xargs -i readlink -f {})

docker run --rm \
    --privileged \
    --env DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.ssh:/home/.ssh \
    -v "$scriptDir/shared/:/projects" \
    -v $WS_PATH:/home/ws \
    --privileged \
    --net=host \
    --hostname $(hostname) \
    -it osrf/ros:humble-gazebo /bin/bash
