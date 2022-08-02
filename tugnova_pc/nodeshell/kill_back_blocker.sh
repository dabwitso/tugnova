#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
${SHELL_DIR}/common.sh

rosnode kill back_blocker
