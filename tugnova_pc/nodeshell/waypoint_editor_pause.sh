#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

rostopic pub -1 /waypoint_editor_pause carctl_msgs/wp_edit_pause_msg "{waypoint: ${1}, status: ${2}}"