#!/bin/bash

BASE_DIR="${HOME}/Autoware/"
SHELL_DIR="${BASE_DIR}/ros/nodeshell"
MONITOR_DIR="${BASE_DIR}/ros/src/carctlsystem/packages/monitoring_health/scripts"
SHELL_LIST="check_cpu.sh check_mem.sh check_proc.sh ping_device.sh"

if [ -z ${ROS_PACKAGE_PATH} ]; then
    ${SHELL_DIR}/common.sh
fi

for SHELL_NAME in ${SHELL_LIST}; do
    RESULT=0
    RESULT=$(ps -ef | grep ${SHELL_NAME} | grep -v grep | wc -l)
    if [ ${RESULT} -eq 0 ]; then
        ${MONITOR_DIR}/${SHELL_NAME} >/dev/null &
    fi
done

roslaunch monitoring_health monitoring_health.launch

#sleep 1000
