#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
roslaunch lane_planner lane_select.launch enablePlannerDynamicSwitch:=False

#sleep 1000
