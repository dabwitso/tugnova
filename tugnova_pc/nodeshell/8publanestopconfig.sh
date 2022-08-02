#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
rostopic pub /config/lane_stop autoware_config_msgs/ConfigLaneStop '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, manual_detection: True}'

#sleep 1000

