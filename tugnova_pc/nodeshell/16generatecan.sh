#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

roslaunch generate_can generate_can.launch $@

#sleep 1000

