#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
source /home/nvidia/ws_livox/devel/setup.bash
echo RunLivox

roslaunch livox_ros_driver livox_lidar.launch $@

#sleep 1000
