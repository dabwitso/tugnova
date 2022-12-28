#!/bin/bash

MODE=$1
LIVOX_IP=$2

EXECUTE_PING_SCRIPT=/home/nvidia/Autoware/ros/nodeshell/ping_livox.sh
EXECUTE_DISPLAY_SCRIPT=/home/nvidia/Autoware/ros/nodeshell/livox_dead_notice.sh

if [ $MODE == "ping" ];
then
  gnome-terminal --command "${EXECUTE_PING_SCRIPT} ${LIVOX_IP}"
else
  gnome-terminal --command "${EXECUTE_DISPLAY_SCRIPT}"
fi


