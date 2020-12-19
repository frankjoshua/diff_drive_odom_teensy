#!/bin/bash

ROS_IP=127.0.0.1
ROS_MASTER_URI=http://$ROS_IP:11311

docker run -d \
    --network="host" \
    --env="ROS_IP=$ROS_IP" \
    --env="ROS_MASTER_URI=$ROS_MASTER_URI" \
    --name ros_master \
    frankjoshua/ros-master

docker run -d \
    --network="host" \
    --env="ROS_IP=$ROS_IP" \
    --env="ROS_MASTER_URI=$ROS_MASTER_URI" \
    --device="/dev/ttyACM0:/dev/arduino" \
    --name ros_serial \
    frankjoshua/ros-serial

docker run -it --rm \
    --network="host" \
    --env="ROS_IP=$ROS_IP" \
    --env="ROS_MASTER_URI=$ROS_MASTER_URI" \
    frankjoshua/ros-master /ros_entrypoint.sh bash

docker stop ros_master
docker stop ros_serial
docker rm ros_master
docker rm ros_serial