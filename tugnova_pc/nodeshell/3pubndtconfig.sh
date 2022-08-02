#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

INIT_POS_GNSS=0
USE_PREDICT_POSE=1
ERROR_THRESHOLD=1.0
RESOLUTION=1.0
STEP_SIZE=0.1
TRANS_EPSILON=0.01
MAX_ITERATIONS=30

ARGS=$@
for ARG in $ARGS[@] ; do
  KEY=$(cut -d'=' -f1 <<<$ARG)
  VALUE=$(cut -d'=' -f2 <<<$ARG)

  if [ $KEY == 'init_pos_gnss:' ]; then
    INIT_POS_GNSS=$VALUE
  elif [ $KEY == 'use_predict_pose:' ]; then
    USE_PREDICT_POSE=$VALUE
  elif [ $KEY == 'error_threshold:' ]; then
    ERROR_THRESHOLD=$VALUE
  elif [ $KEY == 'resolution:' ]; then
    RESOLUTION=$VALUE
  elif [ $KEY == 'step_size:' ]; then
    STEP_SIZE=$VALUE
  elif [ $KEY == 'trans_epsilon:' ]; then
    TRANS_EPSILON=$VALUE
  elif [ $KEY == 'max_iterations:' ]; then
    MAX_ITERATIONS=$VALUE
  fi
done

SRC="/home/nvidia/Autoware/LastPose/"
cd $SRC

FILE="ndt_matching_log.csv"

if [ -f $FILE ]; then

	tgtArray=$(cat $FILE | awk 'END{print $0}')
	IFS=','
	set -- $tgtArray
else
	tgtDummy=("0","0","0","0","0","0","0","0","0","0")
	IFS=','
	set -- $tgtDummy
fi

cd ~/Autoware/ros
source /opt/ros/kinetic/setup.bash
source ./devel/setup.bash

#${5}:x ${6}:y ${7}:z ${8}:roll ${9}:pitch ${10}:Yaw
rostopic pub /config/ndt autoware_config_msgs/ConfigNDT "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, init_pos_gnss: $INIT_POS_GNSS, x: ${5}, y: ${6}, z: ${7}, roll: ${8}, pitch: ${9}, yaw: ${10}, use_predict_pose: $USE_PREDICT_POSE, error_threshold: $ERROR_THRESHOLD, resolution: $RESOLUTION, step_size: $STEP_SIZE, trans_epsilon: $TRANS_EPSILON, max_iterations: $MAX_ITERATIONS}"


