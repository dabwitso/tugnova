#!/bin/bash

BASE_DIR=/home/nvidia/Autoware
API_DIR=${BASE_DIR}/api
ROS_DIR=${BASE_DIR}/ros
SHELL_DIR=${ROS_DIR}/nodeshell
${SHELL_DIR}/common.sh
RTMGR_DIR=${ROS_DIR}/src/util/packages/runtime_manager/scripts
PARAM_YAML=${RTMGR_DIR}/param.yaml

# cd ~/Autoware/ros/
cd ${ROS_DIR}
#export ROS_MASTER_URI=http://192.168.1.77:11311
#export ROS_IP=192.168.1.77
source /opt/ros/kinetic/setup.bash
source ${ROS_DIR}/devel/setup.bash
#roscore

logger ChangeRoute.sh start
echo $1 > ${BASE_DIR}/current_lane_id
rosparam set /current_lane_id $1

courseid=$(cut -d'_' -f 1 <<<$1)
branchid=$(cut -d'_' -f 2 <<<$1)

echo "courseid: $courseid"
echo "branchid: $branchid"

ZIP_DIR=${BASE_DIR}/ros/renkei/${courseid}/ROUTE/${branchid}
ROUTE_FILE=${courseid}_${branchid}.csv
echo ${ZIP_DIR}
if [ ! -e ${ZIP_DIR} ] ; then
  echo "マップもしくはルートの指定誤り"
  exit 1
fi

rm -rf ${BASE_DIR}/route/*
cp -p ${ZIP_DIR}/*${ROUTE_FILE} ${BASE_DIR}/route/route.csv

cd ${SHELL_DIR}


#12waypointloader.sh ${BASE_DIR}/route/route.csv > /dev/null 2>&1 &
gnome-terminal --command "${SHELL_DIR}/12waypointloader.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} waypoint_loader multi_lane_csv`"

# sleep 5
# gnome-terminal --command ${SHELL_DIR}/dicisionmaker.sh
# sleep 15

# gnome-terminal --command ${SHELL_DIR}/pubEngage.sh
# sleep 1

# gnome-terminal --command ${SHELL_DIR}/pubstart.sh
# sleep 1

# gnome-terminal --command ${SHELL_DIR}/pubStop.sh
# sleep 5

xdotool windowsize $(xdotool search --onlyvisible --name firefox) 100% 100%
sleep 2
xdotool windowactivate $(xdotool search --onlyvisible --name firefox)
sleep 2

logger ChangeRoute.sh end
exit 0
