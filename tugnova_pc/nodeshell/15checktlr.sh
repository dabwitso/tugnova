#!/bin/bash
echo 15checktlr

BASE_DIR="${HOME}/Autoware/"
SHELL_DIR="${BASE_DIR}/ros/nodeshell"
MONITOR_DIR="${BASE_DIR}/ros/src/carctlsystem/packages/monitoring_health/scripts"
SHELL_LIST="check_cpu.sh check_mem.sh check_proc.sh ping_device.sh"

if [ -z ${ROS_PACKAGE_PATH} ]; then
    ${SHELL_DIR}/common.sh
fi

roslaunch checktlr checktlr.launch

#sleep 1000
