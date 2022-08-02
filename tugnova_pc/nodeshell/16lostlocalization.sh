#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 16lost_localization

rosrun lost_localization lost_localization

#sleep 1000
