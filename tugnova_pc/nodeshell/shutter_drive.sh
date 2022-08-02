#!/bin/bash

echo "Starting to drive..."
rostopic pub -1 /drive_start_flag std_msgs/Int16 "data: 1"
