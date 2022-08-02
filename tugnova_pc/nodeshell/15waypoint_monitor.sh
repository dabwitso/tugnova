#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 15waypoint_monitor

rosrun waypointmonitor waypointmonitor

#sleep 1000
