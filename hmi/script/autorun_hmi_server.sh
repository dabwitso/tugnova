#!/bin/sh

source /opt/ros/noetic/setup.bash
source /home/ubuntu/hmi_server/ros/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.37:11311
export ROS_IP=192.168.0.30
#
roslaunch launcher hmi_server.launch

echo "$(date): hmi server started" >> /home/ubuntu/hmi_server/ros/log/server.log


