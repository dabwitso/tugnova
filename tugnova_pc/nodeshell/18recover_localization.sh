#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo 18recover_localization

roslaunch recover_localization recover_localization.launch $@
#sleep 1000
