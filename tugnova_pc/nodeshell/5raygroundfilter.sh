#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $@
roslaunch points_preprocessor ray_ground_filter.launch $@

#sleep 1000
