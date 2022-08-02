#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $@
roslaunch autoware_connector vel_pose_connect.launch $@

#sleep 1000
