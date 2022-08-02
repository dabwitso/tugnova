#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
WORK_DIR=${BASE_DIR}/work
TEMP_TOP=/tmp

CUSTOM_YAML=${BASE_DIR}/param/custom.yaml

${SHELL_DIR}/common.sh

LANE_ID_FILE=${BASE_DIR}/current_lane_id
LANE_INFO=$(<${LANE_ID_FILE})
ROUTE_NAME=$(cut -d'_' -f 1 <<<${LANE_INFO})

cd ${SHELL_DIR}
if [ -e "${WORK_DIR}/${ROUTE_NAME}" ]; then
    # delete non-map data & initialize
    ./zip.sh ${ROUTE_NAME} ${WORK_DIR}
    ZIP_FILE=$(<${TEMP_TOP}/ZIP_FILENAME)
    mv ${ZIP_FILE} ${WORK_DIR}/ZIP

    rm -rf ${WORK_DIR}/${ROUTE_NAME}/HOTSPOTS/*
    rm -rf ${WORK_DIR}/${ROUTE_NAME}/ROUTE/*
    touch ${WORK_DIR}/${ROUTE_NAME}/ROUTE/${ROUTE_NAME}_INDEX.csv
else
    # not exist directory
    echo "messageId: api_1_12"
    exit 1
fi

sleep 3

rostopic pub -1 /waypoint_editor_begin carctl_msgs/wp_edit_begin_msg "{mode_2d: ${1}}" 