#!/bin/bash

AUTOWARE_ROUTE_DIR=/home/nvidia/Autoware/ROUTE
GIT_DIR=/home/nvidia/gitlab/routes

# Only copy files if gitlab folder exists
if [ ! -z $GIT_DIR ];
then
  # Ensure folder exists
  if [ -z $AUTOWARE_ROUTE_DIR ];
  then
    mkdir $AUTOWARE_ROUTE_DIR
  fi
  rm $AUTOWARE_ROUTE_DIR/*.csv >&/dev/null
  cp $GIT_DIR/*.csv $AUTOWARE_ROUTE_DIR/
fi

