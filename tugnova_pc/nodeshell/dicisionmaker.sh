#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
roslaunch decision_maker decision_maker.launch $@
#sleep 1000

