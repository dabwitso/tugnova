#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell

cd ${BASE_DIR}

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
source /home/nvidia/Autoware/ros/devel/setup.bash
source /home/nvidia/ws_livox/devel/setup.bash

#export ROS_IP=192.168.0.37
#export ROS_MASTER_URI=http://192.168.0.37:11311
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311


