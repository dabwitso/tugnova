#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $@
roslaunch waypoint_maker waypoint_loader.launch multi_lane_csv:=/home/nvidia/Autoware/ROUTE/routeod.csv $@

#sleep 1000

