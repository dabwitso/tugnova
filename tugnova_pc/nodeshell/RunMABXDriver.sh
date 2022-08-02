#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

roslaunch mabx_connector mabx_connector.launch destination:=172.16.1.1

sleep 10

