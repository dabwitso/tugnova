#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
TEMP_TOP=/tmp
TEMP_WORK_DIR=${TEMP_TOP}/zip-contents

ROUTE_NAME=${1}
TOP_PATH=${2}

DATE=$(date "+%Y%m%d%H%M%S")

cd ${TOP_PATH}

rm -rf ${TEMP_WORK_DIR}
mkdir ${TEMP_WORK_DIR}
cp -r DICTIONARY ${TEMP_WORK_DIR}
cp -r MAPS ${TEMP_WORK_DIR}
cp -r ${ROUTE_NAME} ${TEMP_WORK_DIR}

cd ${TEMP_WORK_DIR}
zip -r ${TEMP_TOP}/${ROUTE_NAME}_${DATE}.zip ./

echo ${TEMP_TOP}/${ROUTE_NAME}_${DATE}.zip > ${TEMP_TOP}/ZIP_FILENAME