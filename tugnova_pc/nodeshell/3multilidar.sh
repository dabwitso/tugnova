#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $@
roslaunch multi_lidar_calibrator multi_lidar_calibrator.launch $@

#sleep 1000

