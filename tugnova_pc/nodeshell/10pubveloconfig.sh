#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $1
rostopic pub /config/velocity_set autoware_config_msgs/ConfigVelocitySet $1

#sleep 1000
