#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
rosrun rviz rviz -d $(rospack find runtime_manager)/../../../.config/rviz/default.rviz

#sleep 1000
