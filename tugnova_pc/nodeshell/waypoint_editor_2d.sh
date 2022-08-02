#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

rostopic pub -1 /waypoint_editor_2d carctl_msgs/wp_edit_2d_msg "{waypoint: ${1}, mode: ${2}}" 
