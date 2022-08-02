#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $@
roslaunch waypoint_follower pure_pursuit.launch $@

#sleep 1000
