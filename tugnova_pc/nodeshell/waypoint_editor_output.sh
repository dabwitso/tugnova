#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

OLDIFS=$IFS
IFS=,

LIST=${2}
OUTPUTS=""

for ITEM in ${LIST[@]}; do
  TARGET_ID=$(cut -d'=' -f 1 <<<${ITEM})
  VALUE=$(cut -d'=' -f 2 <<<${ITEM})

  if [ -n "${OUTPUTS}" ]; then
    OUTPUTS="${OUTPUTS},"
  fi
  OUTPUTS="${OUTPUTS}{target_id: '${TARGET_ID}', value: '${VALUE}'}"
done

IFS=$OLDIFS

rostopic pub -1 /waypoint_editor_output carctl_msgs/wp_edit_output_msg "{waypoint: ${1}, output_targets: [${OUTPUTS}]}"
