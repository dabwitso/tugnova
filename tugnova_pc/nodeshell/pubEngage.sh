#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

rostopic pub -1 /state_cmd std_msgs/String '"engage"'

