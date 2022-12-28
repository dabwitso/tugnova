#!/bin/bash
ROS_DIR=/home/nvidia/Autoware/ros
SHELL_DIR=${ROS_DIR}/nodeshell
HOTSPOT_FILE=${ROS_DIR}/renkei/hotspots/$1
ERROR_FLAG=2

# check if hotspot file exists and process accordingly
if [ -f $HOTSPOT_FILE ];
then
  for (( i=0; i<=1; i++ ));
  do
    rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped < $HOTSPOT_FILE
    sleep 1
  done
else
  rostopic pub -1 /hotspot_error std_msgs/Int16 "data: $ERROR_FLAG"
fi



