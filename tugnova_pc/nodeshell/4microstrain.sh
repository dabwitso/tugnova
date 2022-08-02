#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo $@
roslaunch microstrain_driver 3dm_gx5_15.launch $@

#sleep 1000

