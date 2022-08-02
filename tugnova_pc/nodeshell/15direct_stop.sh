#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 15direct_stop

roslaunch direct_stop direct_stop.launch $@
#sleep 1000
