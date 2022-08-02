#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $@
roslaunch  lidar_localizer  ndt_matching.launch $@

#sleep 1000

