#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
echo ${@:1}

if [ -z $3 ]; then
 #arealistが空の場合
 rosrun map_file points_map_loader $1 $2
else
 #arealistに記載がある場合
 rosrun map_file points_map_loader $1 $3 $2
fi

#sleep 1000

