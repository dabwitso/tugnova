#!/bin/bash

BASE_DIR=/home/nvidia/Autoware
ROS_DIR=${BASE_DIR}/ros


SOURCE_DIR=/home/nvidia/gitlab/hmi_tugnova_msgs/communication_msgs
DESTINATION_DIR=/home/nvidia/Autoware/ros/src/communication/packages


echo "
cleaning folder ${DESTINATION_DIR}...

"
rm -rf ${DESTINATION_DIR}/communication_msgs/

echo "
loading route files from ${SOURCE_DIR} into ${DESTINATION_DIR} ...

"
cp -r ${SOURCE_DIR} ${DESTINATION_DIR}/

read -t 5 -p "

Starting to compile and build messages

"

./${ROS_DIR}/nodeshell/ExitAutoware.sh

cd ${ROS_DIR}

source /opt/ros/kinetic/setup.bash
source ${ROS_DIR}/devel/setup.bash

catkin_make --pkg communication_msgs

read -t 5 -p "
Re-launching Autoware...

"
cd ${BASE_DIR}

./${BASE_DIR}/api/apiserve.sh start


echo "Completed"
