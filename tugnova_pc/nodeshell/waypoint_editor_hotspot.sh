#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

rostopic pub -1 /waypoint_editor_hotspot carctl_msgs/wp_edit_hotspot_msg "{waypoint: ${1}, status: ${2}}" 