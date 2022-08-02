#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

roslaunch runtime_manager setup_vehicle_info.launch $@

sleep 10

