#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
RENKEI_DIR=${BASE_DIR}/ros/renkei
TEMP_TOP=/tmp
RESTORATION_DIR=/home/nvidia/restoration

ROUTE_NAME=$(<${RESTORATION_DIR}/ROUTE_FILE)

rm -rf ${RENKEI_DIR}//DICTIONARY
rm -rf ${RENKEI_DIR}/MAPS
if [ -e ${RESTORATION_DIR}/${ROUTE_NAME} ]; then
    rm -rf ${RENKEI_DIR}/${ROUTE_NAME}
    cp -rfp ${RESTORATION_DIR}/${ROUTE_NAME} ${RENKEI_DIR}
fi


cp -f ${RESTORATION_DIR}/current_lane_id ${BASE_DIR}/current_lane_id
cp -rf ${RESTORATION_DIR}/DICTIONARY ${RENKEI_DIR}
cp -rf ${RESTORATION_DIR}/MAPS ${RENKEI_DIR}

rm -rf ${BASE_DIR}/map/*
cp ${RENKEI_DIR}/${ROUTE_NAME}/MAP/* ${BASE_DIR}/map/
