#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
API_DIR=${BASE_DIR}/api
ROS_DIR=${BASE_DIR}/ros
SHELL_DIR=${ROS_DIR}/nodeshell
PARAM_YAML=${ROS_DIR}/src/util/packages/runtime_manager/scripts/param.yaml
CUSTOM_YAML=${BASE_DIR}/param/custom.yaml

LANE_ID_FILE=${BASE_DIR}/current_lane_id

LANE_INFO=$(<${LANE_ID_FILE})
courseid=$(cut -d'_' -f 1 <<<${LANE_INFO})

ZIP_DIR=${BASE_DIR}/ros/renkei/${courseid}/ROUTE
INDEX_FILE=${ZIP_DIR}/${courseid}_INDEX.csv

if [ -z ${ROS_PACKAGE_PATH} ]; then
  ${SHELL_DIR}/common.sh
fi

SRC=${BASE_DIR}/LastPose

FILES=$(ls ${SRC}/*.csv 2>/dev/null | xargs -i basename {})
RESULT_FILE=dummy.csv

DISTANCE=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} scan_respawn_point distance)
ANGLE=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} scan_respawn_point angle)
POSITION_CHECK_TIMER=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} scan_respawn_point position_check_timer)
HOTSPOT_X=0
HOTSPOT_Y=0
HOTSPOT_Z=0
HOTSPOT_YAW=0


COUNT=0
RESULT=-1
MINIMUN_SCORE=2147483646
THRESHOLD=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} scan_respawn_point threshold)

doInitialpose() {
  cd ${SRC}

  if [ -f $1 ]; then
    tgtArray=$(cat $1 | awk 'END{print $0}')
    IFS=','
    set -- $tgtArray
  else
    tgtDummy=("0","0","0","0","0","0","0","0","0","0")
    IFS=','
    set -- $tgtDummy
  fi

  cd ${ROS_DIR}/devel/lib/ctlcar

  #${5}:x ${6}:y ${7}:z ${8}:roll ${9}:pitch ${10}:Yaw
  HOTSPOT_X=${5}
  HOTSPOT_Y=${6}
  HOTSPOT_Z=${7}
  HOTSPOT_YAW=${10}
  ./pub_initialpose ${5} ${6} ${7} ${8} ${9} ${10}
}

startBackBlocker() {
  echo "== RUN Back Blocker =="
  ${SHELL_DIR}/respawn_node.py back_blocker
}

stopBackBlocker() {
  rosnode kill back_blocker
}

startVehicleWaypointDistanceChecker() {
  echo "== RUN Vehicle Waypoint Distance Checker =="
  ${SHELL_DIR}/respawn_node.py vehicle_waypoint_distance_checker
}

stopVehicleWaypointDistanceChecker() {
  rosnode kill vehicle_waypoint_distance_checker
}

sendLostLocalization() {
  rostopic pub -1 /remote_cmd autoware_msgs/RemoteCmd "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, vehicle_cmd: {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, steer_cmd: {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, steer: 0}, accel_cmd: {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, accel: 0}, brake_cmd: {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, brake: 0}, lamp_cmd: {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, l: 0, r: 0}, gear: 0, mode: 0, twist_cmd: {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}, ctrl_cmd: {linear_velocity: 0.0, linear_acceleration: 0.0, steering_angle: 0.0}, emergency: 1}, control_mode: 4}"
}

setFrontBrowserUI() {
  xdotool windowsize $(xdotool search --onlyvisible --name firefox) 100% 100%
  sleep 2
  xdotool windowactivate $(xdotool search --onlyvisible --name firefox)
  sleep 2
}


echo "== SCAN START == "

stopBackBlocker
stopVehicleWaypointDistanceChecker

for FILE in ${FILES[@]}; do
  if [ ${FILE} = "ndt_matching_log.csv" ]; then
    continue
  fi

  doInitialpose ${FILE}
  SCORE=$(timeout 5 ./sub_ndt_stat)

  echo ${SCORE}

  echo ${SCORE} | grep WARN
  if [ $? -eq 0 ]; then
    continue
  fi

  if [ $(echo "$SCORE < $MINIMUN_SCORE" | bc) == 1 ] && [ $(echo "$SCORE < $THRESHOLD" | bc) == 1 ]; then
    RESULT_FILE=${FILE}
    RESULT=$COUNT
    MINIMUN_SCORE=$SCORE
    echo "== UPDATE MINUMUM SCORE =="
    echo $RESULT $MINIMUN_SCORE
  fi
  let COUNT++
done
if [ "$RESULT" == -1 ]; then
  logger -t scan_respawn_point NG - NO HIT
  sendLostLocalization
  echo "messageId: api_1_2"
  # Don't run "back_blocker" for "SENDING INCORRECT INFO."
  startVehicleWaypointDistanceChecker
  setFrontBrowserUI
  exit 1
fi

echo "== SCAN FINISH =="
echo "RESULT: " $RESULT

#echo "== PhoneOrderFlag On =="
#${ROS_DIR}/devel/lib/phoneorder/run_phoneflg

echo "== INITIALPOSE =="
doInitialpose ${RESULT_FILE}


echo "== POSITION CHECK =="
sleep ${POSITION_CHECK_TIMER}
POSITION_CHECK=$(rosrun ctlcar hotspot_position_check _x:=${HOTSPOT_X} _y:=${HOTSPOT_Y} _z:=${HOTSPOT_Z} _yaw:=${HOTSPOT_YAW} _distance:=${DISTANCE} _angle:=${ANGLE})
echo ${POSITION_CHECK}

if [ ${POSITION_CHECK} != "OK" ]; then
  logger -t scan_respawn_point ${POSITION_CHECK}
  sendLostLocalization
  echo "messageId: api_1_2"
  # Don't run "back_blocker" for "SENDING INCORRECT INFO."
  startVehicleWaypointDistanceChecker
  setFrontBrowserUI
  exit 1
fi


echo "== VelocitySet =="
rosnode kill \/velocity_set
python ~/Autoware/ros/nodeshell/respawn_node.py velocity_set

echo "== CheckChangeRoute =="
rosnode kill \/check_change_route
python ~/Autoware/ros/nodeshell/respawn_node.py check_change_route

echo "== ChangeRoute =="
OLDIFS=$IFS 
IFS=,
if [ -f ${INDEX_FILE} ]; then
  for ITEM in `cat ${INDEX_FILE}` ; do
    ITEM=$(sed 's/\r//g' <<<$ITEM)
    FILENAME=$(cut -d':' -f 1 <<<${ITEM})
    ID=$(cut -d':' -f 2 <<<${ITEM})

    if [ ${FILENAME} = ${RESULT_FILE} ]; then
      ${SHELL_DIR}/ChangeRoute.sh ${courseid}_${ID}
      break
    fi
  done
fi
IFS=$OLDIFS

startBackBlocker
startVehicleWaypointDistanceChecker

setFrontBrowserUI

echo "== publish scan_respawn_result == "
rostopic pub -1 /scan_respawn_result std_msgs/Int16 1

sleep 5
BUFF=`grep acquired ${API_DIR}/VEHICLE_STATUS`
if [ $? -eq 1 ]; then
  logger -t scan_respawn_point NG - NDT LOST
  # No "sendLostLocalization" function call required. Because this script has already detected the LOST status.
  echo "messageId: api_1_2"
  exit 1
fi
echo "messageId: api_0_0"
