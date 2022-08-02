#!/bin/bash


BASE_DIR=/home/nvidia/Autoware
ROS_DIR=${BASE_DIR}/ros
SHELL_DIR=${ROS_DIR}/nodeshell
CUSTOM_YAML=${BASE_DIR}/param/custom.yaml

STATE=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} ctl_led auto_state_code)


${SHELL_DIR}/LED_ctl.sh $STATE           > /dev/null 2>&1 &
#gnome-terminal --command "${SHELL_DIR}/LED_ctl.sh $STATE"

