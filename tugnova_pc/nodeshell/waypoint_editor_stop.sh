#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

OLDIFS=$IFS
IFS=,

LIST=${3}
INPUTS=""

for ITEM in ${LIST[@]}; do
  if [ -n "${INPUTS}" ]; then
    INPUTS="${INPUTS},"
  fi
  INPUTS="${INPUTS}'${ITEM}'"
done

IFS=$OLDIFS

rostopic pub -1 /waypoint_editor_stop carctl_msgs/wp_edit_stop_msg "{waypoint: ${1}, input_targets: [${INPUTS}], status: ${2}}" 
