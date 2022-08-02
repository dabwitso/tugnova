#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
roslaunch waypoint_follower twist_filter.launch

#sleep 1000
