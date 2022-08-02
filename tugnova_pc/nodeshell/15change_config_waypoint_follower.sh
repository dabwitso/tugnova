#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 15change_config_waypoint_follower

rosrun change_config_waypoint_follower change_config_waypoint_follower

#sleep 1000
