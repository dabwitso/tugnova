#!/bin/bash

./respawn_node.py back_blocker

sleep 5

rostopic pub -1 /scan_respawn_result std_msgs/Int16 "data: 1"
