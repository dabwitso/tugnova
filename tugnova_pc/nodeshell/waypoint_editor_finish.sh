#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

rostopic pub -1 /waypoint_editor_finish carctl_msgs/wp_edit_finish_msg "{}" 