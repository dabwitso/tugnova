#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

roslaunch waypoint_maker waypoint_saver.launch save_finename:=/home/nvidia/Autoware/ros/logs/route_standard_version.csv $@