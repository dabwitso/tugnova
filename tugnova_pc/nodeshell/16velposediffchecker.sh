#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
API_DIR=${BASE_DIR}/api
ROS_DIR=${BASE_DIR}/ros
SHELL_DIR=${ROS_DIR}/nodeshell
VEHICLE_INFO=$1
${SHELL_DIR}/common.sh

MEDIAN_DISTANCE=2.0
ARGS=${@:2}
for ARG in $ARGS[@] ; do
  KEY=$(cut -d'=' -f1 <<<$ARG)
  VALUE=$(cut -d'=' -f2 <<<$ARG)

  if [ $KEY == 'diff_position_median_threshold_meter:' ]; then
    MEDIAN_DISTANCE=$VALUE
    break
  fi
done

echo "median distance: $MEDIAN_DISTANCE"

distance=`echo "scale=5; $MEDIAN_DISTANCE + 0.5" | bc`
echo "distance: $distance"


median_rad=`python ${SHELL_DIR}/atan.py $MEDIAN_DISTANCE $VEHICLE_INFO`
echo "median_rad: $median_rad"

rad=`echo "scale=5; $median_rad + 0.05" | bc`
echo "rad: $rad" 


roslaunch vel_pose_diff_checker vel_pose_diff_checker.launch diff_position_threshold_meter:=$distance diff_angle_threshold_rad:=$rad diff_angle_median_threshold_rad:=$median_rad $ARGS

#sleep 1000

