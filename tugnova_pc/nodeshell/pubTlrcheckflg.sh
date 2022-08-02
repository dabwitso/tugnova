#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
rostopic pub /TlrCheckFlg std_msgs/Int16 "data: 1"

