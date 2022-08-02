#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $1
rostopic pub /config/lane_select autoware_config_msgs/ConfigLaneSelect $1

#sleep 1000
