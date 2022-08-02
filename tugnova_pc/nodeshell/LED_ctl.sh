#!/bin/bash


BASE_DIR=/home/nvidia/Autoware
ROS_DIR=${BASE_DIR}/ros
SHELL_DIR=${ROS_DIR}/nodeshell
CUSTOM_YAML=${BASE_DIR}/param/custom.yaml

LED_IP=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} ctl_led led_ip)
LED_PORT=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} ctl_led led_port)

STATE=$1


echo $STATE | xxd -r -p | nc $LED_IP $LED_PORT
