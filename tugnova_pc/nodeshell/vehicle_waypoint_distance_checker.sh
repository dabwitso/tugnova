#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
${SHELL_DIR}/common.sh

roslaunch vehicle_waypoint_distance_checker vehicle_waypoint_distance_checker.launch $@
