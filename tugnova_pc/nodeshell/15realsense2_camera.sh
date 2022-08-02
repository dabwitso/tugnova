#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 15realsense2_camera
source ~/catkin_ws/install/setup.bash

roslaunch realsense2_camera rs_camera.launch

#sleep 1000
