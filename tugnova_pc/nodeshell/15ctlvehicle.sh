#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 15ctlvehicle

rosrun ctlvehicle ctlvehicle

#sleep 1000
