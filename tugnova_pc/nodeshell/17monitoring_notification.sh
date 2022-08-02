#!/bin/bash

BASE_DIR="${HOME}/Autoware/"
SHELL_DIR="${BASE_DIR}/ros/nodeshell"

if [ -z ${ROS_PACKAGE_PATH} ]; then
    ${SHELL_DIR}/common.sh
fi

roslaunch monitoring_health monitoring_notification.launch

#sleep 1000
