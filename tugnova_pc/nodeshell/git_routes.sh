#!/bin/bash

NODESHELL_DIR=/home/nvidia/Autoware/ros/nodeshell
NEW_SETUP=$1

bash $NODESHELL_DIR/load_save_git.sh routes ${NEW_SETUP:=not_initial_setup}
