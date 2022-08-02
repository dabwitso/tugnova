#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $@
roslaunch points_preprocessor points_concat_filter.launch $@

#sleep 1000

