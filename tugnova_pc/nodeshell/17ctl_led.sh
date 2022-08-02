#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 17ctl_led

rosrun ctl_led ctl_led
#sleep 1000
