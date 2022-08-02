#!/bin/bash

THRESHOLD=30.0

rostopic pub -r 10 /battery_threshold std_msgs/Float64 "data: ${THRESHOLD}"
