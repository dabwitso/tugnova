#!/bin/bash

cd /home/nvidia/Autoware/ros/nodeshell/
rostopic pub -1 /reset_object_detection std_msgs/Int16 "data: 1"
./respawn_node.py waypoint_loader
