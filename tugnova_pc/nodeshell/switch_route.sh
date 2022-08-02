#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
WORK_DIR=${BASE_DIR}/work

${SHELL_DIR}/common.sh

ROUTE_NAME=${1}

cd ${SHELL_DIR}
if [ -z "${ROUTE_NAME}" ] || [ ! -e "${WORK_DIR}/${ROUTE_NAME}" ]; then
    # not exist directory
    echo "messageId: api_1_12"
    exit 1
fi

echo "${ROUTE_NAME}_1" > ${BASE_DIR}/current_lane_id