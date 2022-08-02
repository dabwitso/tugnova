#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 15update_map_route

rosrun update_map_route update_map_route

#sleep 1000
