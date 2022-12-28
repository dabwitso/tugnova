#!/bin/bash

GIT_DIR=/home/nvidia/gitlab
PARAM_DIR=$GIT_DIR/params
AUTOWARE_PARAM_DIR=/home/nvidia/Autoware/param
RUNTIME_MGR_PARAM=/home/nvidia/Autoware/ros/src/util/packages/runtime_manager/scripts

if [[ ! -d $GIT_DIR ]];
then
  mkdir $GIT_DIR
fi

if [[ ! -d $PARAM_DIR ]];
then
  echo "ERROR: $PARAM_DIR not found. Cannot load parameter. Terminating..."
  exit 1
fi


rm $AUTOWARE_PARAM_DIR/param.yaml >&/dev/null
rm $AUTOWARE_PARAM_DIR/custom.yaml >&/dev/null
rm $AUTOWARE_PARAM_DIR/device.lst >&/dev/null

cp $PARAM_DIR/custom.yaml $AUTOWARE_PARAM_DIR
cp $PARAM_DIR/device.lst $AUTOWARE_PARAM_DIR

# check if param missing from runtime_manager folder
if [[ ! -f "$RUNTIME_MGR_PARAM/param.yaml" ]];
then
  cp $PARAM_DIR/param.yaml $RUNTIME_MGR_PARAM
  cd $AUTOWARE_PARAM_DIR
  # create shortcut
  ln -s $RUNTIME_MGR_PARAM/param.yaml param.yaml
fi

echo "

Done loading custom.yaml and device.lst into Autoware/param
"
