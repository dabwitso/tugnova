#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 16generate_vehicle_twist

rosrun generate_vehicle_twist generate_vehicle_twist

#sleep 1000
