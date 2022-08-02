#!/bin/bash

BASE_DIR=/home/nvidia/Autoware
export $(cat ${BASE_DIR}/param/RunSystem.env | grep -v '^#' | xargs)

API_DIR=${BASE_DIR}/api
ROS_DIR=${BASE_DIR}/ros
SHELL_DIR=${ROS_DIR}/nodeshell
RTMGR_DIR=${ROS_DIR}/src/util/packages/runtime_manager/scripts
${SHELL_DIR}/common.sh

PARAM_YAML=${RTMGR_DIR}/param.yaml
CUSTOM_YAML=${BASE_DIR}/param/custom.yaml

cd ${ROS_DIR}

source /opt/ros/kinetic/setup.bash
source ${ROS_DIR}/devel/setup.bash

${SHELL_DIR}/ExitAutoware.sh

ROUTE_NAME=${1}
echo "${ROUTE_NAME}_1" > ${BASE_DIR}/current_lane_id

rm -rf ${BASE_DIR}/map/*
cp ${BASE_DIR}/work/${ROUTE_NAME}/MAP/* ${BASE_DIR}/map/

ZIP_DIR_TOP=${BASE_DIR}/work
ZIP_DIR_LANE=${ZIP_DIR_TOP}/${ROUTE_NAME}/ROUTE
if [ ! -e ${ZIP_DIR_LANE} ] ; then
  echo "マップもしくはルートの指定誤り"
  logger "マップもしくはルートの指定誤り"
  exit 1
fi


gnome-terminal --command ./run


cd ${SHELL_DIR}
sleep 5
rosparam set /current_lane_id ${ROUTE_NAME}_1

#arealistファイルが空だった場合分割地図読み込み機能を使わない
AREA_LIST_FILE=${ZIP_DIR_TOP}/${ROUTE_NAME}/AREA_LIST/arealists.txt
POINT_MAP_AREA=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} readmap point_map_area)
DIVIDE_POINT_MAP_AREA=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} readmap divide_point_map_area)
POINT_MAP_PCP_FILES=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} readmap point_map_pcp_files)
if [ ! -s $AREA_LIST_FILE ]; then
  gnome-terminal --command "${SHELL_DIR}/1readmap.sh $POINT_MAP_AREA $POINT_MAP_PCP_FILES" 
else
  gnome-terminal --command "${SHELL_DIR}/1readmap.sh $DIVIDE_POINT_MAP_AREA $POINT_MAP_PCP_FILES $AREA_LIST_FILE"
fi
sleep 3

${SHELL_DIR}/2readtf.sh           > /dev/null 2>&1 &
${SHELL_DIR}/2readvelodynetf.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} setup_tf`   > /dev/null 2>&1 &

#lidarの構成によってlidar起動コマンドを変更する
USE_LIDAR=$(python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} lidar use_lidar)

if [ $USE_LIDAR = "velodyne" ]; then
  ${SHELL_DIR}/3runvlp16.sh       > /dev/null 2>&1 &
elif [ $USE_LIDAR = "os1_32_gen2" -o $USE_LIDAR = "os1_16_gen2" ]; then
  ${SHELL_DIR}/RunOusterTFGen2.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} ouster_lidar`    > /dev/null 2>&1 &
elif [ $USE_LIDAR = "os1_16_gen1" ]; then
  ${SHELL_DIR}/RunOusterTFGen1.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} ouster_lidar`    > /dev/null 2>&1 &
fi

#${SHELL_DIR}/RunTwoLidar.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} velodyne_lidar`  > /dev/null 2>&1 &
#gnome-terminal --command "${SHELL_DIR}/RunOuster.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} ouster_lidar`"
#${SHELL_DIR}/RunOusterTF.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} ouster_lidar`    > /dev/null 2>&1 &
#${SHELL_DIR}/RunLivox.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} livox_lidar`    > /dev/null 2>&1 &
#${SHELL_DIR}/LivoxTF.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} livox_lidar_tf`    > /dev/null 2>&1 &
sleep 7

#gnome-terminal --command "${SHELL_DIR}/3concatfilter.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} points_concat_filter`"
#sleep 7

${SHELL_DIR}/3voxcelgridfilter.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} voxel_grid_filter`  > /dev/null 2>&1 &
rm -f /home/nvidia/Autoware/LastPose/ndt_matching_log.csv
${SHELL_DIR}/3pubndtconfig.sh     `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} ndt_matching`    > /dev/null 2>&1 &
sleep 7

${SHELL_DIR}/4ndtmatching.sh         `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} ndt_matching`         > /dev/null 2>&1 &
gnome-terminal --command "${SHELL_DIR}/4ndtmatching_monitor.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} ndt_matching_monitor`"
${SHELL_DIR}/4runrviz.sh          > /dev/null 2>&1 &
sleep 7

${SHELL_DIR}/7velposeconnect.sh      `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} vel_pose_connect`    > /dev/null 2>&1 &

sleep 5

gnome-terminal --command "${SHELL_DIR}/waypoint_editor.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} waypoint_editor` `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} waypoint_editor_change_route` `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} waypoint_editor_curve`"
sleep 3
gnome-terminal --command "${SHELL_DIR}/waypoint_saver.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} waypoint_saver save_filename`"

sleep 3
gnome-terminal --command ${SHELL_DIR}/16lostlocalization.sh

sleep 5
cp -p ${BASE_DIR}/param/monitoring_health_route_creator.yaml ${ROS_DIR}/src/carctlsystem/packages/monitoring_health/scripts/monitoring_health.yaml
gnome-terminal --command ${SHELL_DIR}/17monitoring_health.sh
sleep 2
gnome-terminal --command ${SHELL_DIR}/17monitoring_notification.sh
sleep 2
gnome-terminal --command ${SHELL_DIR}/17ping_node.sh
sleep 2

xdotool windowsize $(xdotool search --onlyvisible --name firefox) 100% 100%
sleep 2
xdotool windowactivate $(xdotool search --onlyvisible --name firefox)
sleep 2

exit 0
