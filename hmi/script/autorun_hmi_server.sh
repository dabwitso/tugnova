#!/bin/bash

BASE_DIR=/home/ubuntu/hmi

source /opt/ros/noetic/setup.bash
source ${BASE_DIR}/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.37:11311
export ROS_IP=192.168.0.31
LOG_DIR=${BASE_DIR}/log
#
roslaunch launcher hmi_server.launch

if [[ ! -d  $LOG_DIR ]];
then
  mkdir $LOG_DIR
fi

echo "$(date): hmi server started" >> ${LOG_DIR}/server.log


