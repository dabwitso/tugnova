#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 15check_change_route

rosrun check_change_route check_change_route

#sleep 1000
