#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
rostopic pub /config/waypoint_follower autoware_config_msgs/ConfigWaypointFollower $1

#sleep 1000
