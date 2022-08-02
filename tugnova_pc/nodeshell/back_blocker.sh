#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
${SHELL_DIR}/common.sh

roslaunch back_blocker back_blocker.launch $@
