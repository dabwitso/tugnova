#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
source /home/nvidia/ws_livox/devel/setup.bash

roslaunch livox_ros_driver tf_velodyne_livox.launch $@


#sleep 1000
