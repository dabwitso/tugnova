#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo RunOuster


roslaunch ouster_ros_1_13_1 os1.launch $@

#sleep 1000
