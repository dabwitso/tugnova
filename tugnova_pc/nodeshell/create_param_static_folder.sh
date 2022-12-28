#!/bin/bash

AUTOWARE_BASE_DIR=/home/nvidia/Autoware
PARAM_DIR=$AUTOWARE_BASE_DIR/param
RUNTIME_MGR_DIR=$AUTOWARE_BASE_DIR/ros/src/util/packages/runtime_manager/scripts
PARAM_BACKUP_DIR=/home/nvidia/gitlab/params

if [[ ! -d $PARAM_BACKUP_DIR ]];
then
  mkdir -p $PARAM_BACKUP_DIR
fi

# no sense trying to make backup of non-existent files
if [[ ! -d $PARAM_DIR ]];
then
  echo "Error: $PARAM_DIR not found. Aborting operation..."
  exit 1
fi

cp $PARAM_DIR/custom.yaml $PARAM_BACKUP_DIR/
cp $PARAM_DIR/device.lst $PARAM_BACKUP_DIR/
cp $PARAM_DIR/param.yaml $PARAM_BACKUP_DIR/

# check if param file found in runtime_manager file
if [[ ! -f "$RUNTIME_MGR_DIR/param.yaml" ]];
then
  cp $PARAM_DIR/param.yaml $RUNTIME_MGR_DIR/
fi

echo "

Done setting up param backup folder
"
