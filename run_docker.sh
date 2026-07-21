#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
xhost +local:docker
docker run \
    -it --rm \
    $VOLUMES \
    -v ${XSOCK}:${XSOCK} \
    -v ${XAUTH}:${XAUTH} \
    -v $HOME/.ssh:/root/.ssh:ro \
    -v ./ros_ws:/ros2_ws \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=${XAUTH} \
    -e ROS_DOMAIN_ID=11 \
    -e TURTLEBOT3_MODEL=burger \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    --name="drone_herding" \
    drone_herding:1.0
xhost -local:docker