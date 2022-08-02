#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

roslaunch ouster_ros tf_velodyne_os1.launch $@


#sleep 1000
