#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

roslaunch ctlreduce ctlreduce.launch $@

#sleep 1000
