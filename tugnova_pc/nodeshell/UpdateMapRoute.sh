#!/bin/bash

BASE_DIR=/home/nvidia/Autoware
ROS_DIR=${BASE_DIR}/ros
${ROS_DIR}/nodeshell/common.sh

cd ${ROS_DIR}
source /opt/ros/kinetic/setup.bash
source ${ROS_DIR}/devel/setup.bash

SHELL_DIR=${ROS_DIR}/nodeshell
YAML_FILE=${BASE_DIR}/param/custom.yaml
url=$(python ${SHELL_DIR}/load_param.py  ${YAML_FILE} checksrv post_url)
vehicleId=$(python ${SHELL_DIR}/load_param.py  ${YAML_FILE} checksrv car_vehicleId)
zip_url=${url}/download/${vehicleId}

TEMP_TOP=/tmp
TEMP_DL_DIR=${TEMP_TOP}/dl
TEMP_WORK_DIR=${TEMP_TOP}/update-contents
RENKEI_DIR=${ROS_DIR}/renkei



rm -rf ${TEMP_DL_DIR}
rm -rf ${TEMP_WORK_DIR}
mkdir ${TEMP_DL_DIR}
mkdir ${TEMP_WORK_DIR}

wget --content-disposition $zip_url -P ${TEMP_DL_DIR}
wget_result=$?


max_retry=10
for i in $(seq 1 $max_retry); do
  if [ $wget_result -ne 0 ]; then
    wget --content-disposition $zip_url -P ${TEMP_DL_DIR}
    wget_result=$?
  else
    break
  fi
 
  if [ $i = 10 ]; then
    exit 1
  fi
done

unzip ${TEMP_DL_DIR}/* -d ${TEMP_WORK_DIR}
if [ $? -ne 0 ]; then 
  exit 2
fi

mv ${TEMP_DL_DIR}/* ${RENKEI_DIR}/ZIP
mv ${RENKEI_DIR}/ZIP ${TEMP_WORK_DIR}
rm -rf ${RENKEI_DIR}/*
mv ${TEMP_WORK_DIR}/* ${RENKEI_DIR}

exit 0
