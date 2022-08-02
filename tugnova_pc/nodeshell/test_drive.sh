#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
WORK_DIR=${BASE_DIR}/work
RENKEI_DIR=${BASE_DIR}/ros/renkei
TEMP_TOP=/tmp
RESTORATION_DIR=/home/nvidia/restoration

CUSTOM_YAML=${BASE_DIR}/param/custom.yaml

${SHELL_DIR}/common.sh

ROUTE_NAME=${1}

if [ -z "${ROUTE_NAME}" ] || [ ! -e "${WORK_DIR}/${ROUTE_NAME}" ]; then
    # not exist directory
    echo "messageId: api_1_12"
    exit 1
fi

# backup restoration data
if [ -e "${RESTORATION_DIR}" ]; then
    rm -rf ${RESTORATION_DIR}
fi

mkdir -p ${RESTORATION_DIR}
cp -fp ${BASE_DIR}/current_lane_id ${RESTORATION_DIR}
cp -rfp ${RENKEI_DIR}/DICTIONARY ${RESTORATION_DIR}
cp -rfp ${RENKEI_DIR}/MAPS ${RESTORATION_DIR}
if [ -e ${RENKEI_DIR}/${ROUTE_NAME} ]; then
    cp -rfp ${RENKEI_DIR}/${ROUTE_NAME} ${RESTORATION_DIR}
    echo ${ROUTE_NAME} > ${RESTORATION_DIR}/ROUTE_FILE
fi

echo "${ROUTE_NAME}_1" > ${BASE_DIR}/current_lane_id

rm -rf ${RENKEI_DIR}/${ROUTE_NAME}
cp -r ${WORK_DIR}/DICTIONARY ${RENKEI_DIR}
cp -f ${WORK_DIR}/MAPS ${RENKEI_DIR}
cp -r ${WORK_DIR}/${ROUTE_NAME} ${RENKEI_DIR}

rm -rf ${BASE_DIR}/map/*
cp ${RENKEI_DIR}/${ROUTE_NAME}/MAP/* ${BASE_DIR}/map/
