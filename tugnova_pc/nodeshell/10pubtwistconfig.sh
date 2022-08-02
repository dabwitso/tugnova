#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $1
rostopic pub /config/twist_filter autoware_config_msgs/ConfigTwistFilter $1

#sleep 1000

