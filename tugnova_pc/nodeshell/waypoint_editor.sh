#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
${SHELL_DIR}/common.sh

LANE_ID_FILE=${BASE_DIR}/current_lane_id
LANE_INFO=$(<${LANE_ID_FILE})
ROUTE_NAME=$(cut -d'_' -f 1 <<<${LANE_INFO})

roslaunch waypoint_editor waypoint_editor.launch name:=${ROUTE_NAME} $@
