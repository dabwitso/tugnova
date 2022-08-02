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

logger RunSystem.sh start

INI_LANE_ID=$1"_1"
echo ${INI_LANE_ID} > ${BASE_DIR}/current_lane_id

TEST_DRIVE=0
if [[ -n $2 && $2 -eq 1 ]]; then
  TEST_DRIVE=$2
fi

#rm -rf ${BASE_DIR}/map/*
#cp ${BASE_DIR}/ros/renkei/$1/MAP/* ${BASE_DIR}/map/

courseid=$(cut -d'_' -f 1 <<<${INI_LANE_ID})
branchid=$(cut -d'_' -f 2 <<<${INI_LANE_ID})

echo "courseid: $courseid"
echo "branchid: $branchid"

ZIP_DIR_TOP=${BASE_DIR}/ros/renkei
ZIP_DIR_LANE=${ZIP_DIR_TOP}/${courseid}/ROUTE
ZIP_DIR=${ZIP_DIR_LANE}/${branchid}
LASTPOSE_DIR=${BASE_DIR}/LastPose

#ROUTE_FILE=${courseid}_${branchid}.csv

logger RunSystem.sh start:0
if [ ! -e ${ZIP_DIR} ] ; then
  echo "マップもしくはルートの指定誤り"
  logger "マップもしくはルートの指定誤り"
  exit 1
fi

#rm -rf ${BASE_DIR}/route/*
#cp -p ${ZIP_DIR}/*${ROUTE_FILE} ${BASE_DIR}/route/route.csv
# load kamigo routes from git folder

gnome-terminal --command ${SHELL_DIR}/load_kamigo_routes.sh

cp -p ${ZIP_DIR_TOP}/DICTIONARY/DICTIONARY ${API_DIR}/DICTIONARY

# 前回格納した自己位置取得用ファイルを除去
mv ${LASTPOSE_DIR}/ndt_matching_log.csv ${LASTPOSE_DIR}/ndt_matching_log.csv.bak
rm -f ${LASTPOSE_DIR}/*.csv
mv ${LASTPOSE_DIR}/ndt_matching_log.csv.bak ${LASTPOSE_DIR}/ndt_matching_log.csv

# 自己位置取得用ファイルをINDEXを参照しコピー
INDEX_FILE=${ZIP_DIR_LANE}/${courseid}_INDEX.csv
if [ -f ${INDEX_FILE} ]; then
  INDEX=$(<${INDEX_FILE})

  FILES=${INDEX//,/ }
  for FILE_ in ${FILES[@]} ; do
    # 自己位置取得用ファイル名の抽出
    FILE=`echo ${FILE_} |awk -F: '{print $1}'`
    HOTSPOT=${ZIP_DIR_TOP}/${courseid}/HOTSPOTS/${FILE}
    if [ -f ${HOTSPOT} ]; then
      cp -p ${HOTSPOT} ${LASTPOSE_DIR}
    fi
  done
fi


logger RunSystem.sh start:1
# Process initialize
CMDLIST="twist_monitor.sh location_monitor.sh 4runrviz.sh"
for CMD in $CMDLIST ;do
  ps -ef |grep ${CMD} |grep -v grep |awk '{print $2}' > /tmp/PID.lst
  while read PID ;do
    if [ "${PID}" != '' ]; then
      echo "${CMD} process kill,PID:${PID}"
      kill ${PID}
    fi
  done < /tmp/PID.lst
done

logger RunSystem.sh start:2
gnome-terminal --command ./run


cd ${SHELL_DIR}
sleep 5
rosparam set /current_lane_id ${INI_LANE_ID}

logger RunSystem.sh start:3
#arealistファイルが空だった場合分割地図読み込み機能を使わない
AREA_LIST_FILE=${ZIP_DIR_TOP}/${courseid}/AREA_LIST/arealists.txt
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

${SHELL_DIR}/2pubvoxcelgridfilterconfig.sh `python ${SHELL_DIR}/generate_topic.py ${PARAM_YAML} voxel_grid_filter`    > /dev/null 2>&1 &
sleep 7

${SHELL_DIR}/3voxcelgridfilter.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} voxel_grid_filter`  > /dev/null 2>&1 &
${SHELL_DIR}/5raygroundfilter.sh  `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} ray_ground_filter` `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} ray_ground_filter`  > /dev/null 2>&1 &
${SHELL_DIR}/3pubndtconfig.sh     `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} ndt_matching`    > /dev/null 2>&1 &
sleep 7

${SHELL_DIR}/16setupvehicleinfo.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} vehicle_info`          > /dev/null 2>&1 &
sleep 3
gnome-terminal --command "${SHELL_DIR}/16generatecan.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} generate_can`"
sleep 7

gnome-terminal --command ${SHELL_DIR}/4can2odom.sh

logger RunSystem.sh start:4
${SHELL_DIR}/4ndtmatching.sh         `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} ndt_matching`         > /dev/null 2>&1 &
gnome-terminal --command "${SHELL_DIR}/4ndtmatching_monitor.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} ndt_matching_monitor`"
${SHELL_DIR}/4runrviz.sh          > /dev/null 2>&1 &
sleep 7

${SHELL_DIR}/7velposeconnect.sh      `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} vel_pose_connect`    > /dev/null 2>&1 &
${SHELL_DIR}/8publaneselectconfig.sh `python ${SHELL_DIR}/generate_topic.py ${PARAM_YAML} lane_select`    > /dev/null 2>&1 &
${SHELL_DIR}/8publaneruleconfig.sh   `python ${SHELL_DIR}/generate_topic.py ${PARAM_YAML} lane_rule`      > /dev/null 2>&1 &
${SHELL_DIR}/8publanestopconfig.sh > /dev/null 2>&1 &

${SHELL_DIR}/12waypointloader.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} waypoint_loader multi_lane_csv` > /dev/null 2>&1 &
sleep 7

logger RunSystem.sh start:5
${API_DIR}/location_monitor.sh     > /dev/null 2>&1 &

${SHELL_DIR}/9lanerule.sh          > /dev/null 2>&1 &
${SHELL_DIR}/9lanestop.sh          > /dev/null 2>&1 &
${SHELL_DIR}/9laneselect.sh        > /dev/null 2>&1 &
${SHELL_DIR}/10pubtwistconfig.sh   `python ${SHELL_DIR}/generate_topic.py ${PARAM_YAML} twist_filter` > /dev/null 2>&1 &
${SHELL_DIR}/10pubveloconfig.sh    `python ${SHELL_DIR}/generate_topic.py ${PARAM_YAML} velocity_set use_crosswalk_detection,enable_multiple_crosswalk_detection,points_topic`    > /dev/null 2>&1 &
${SHELL_DIR}/10pubwayconfig.sh     `python ${SHELL_DIR}/generate_topic.py ${PARAM_YAML} pure_pursuit is_linear_interpolation,publishes_for_steering_robot`                        > /dev/null 2>&1 &
sleep 7

${SHELL_DIR}/10obstacleaboid.sh    `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} astar_avoid`       > /dev/null 2>&1 &
${SHELL_DIR}/11velocityset.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} velocity_set` `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} velocity_set` > /dev/null 2>&1 &
${SHELL_DIR}/13purepursuit.sh `python ${SHELL_DIR}/load_yaml.py ${PARAM_YAML} pure_pursuit` > /dev/null 2>&1 &
${SHELL_DIR}/14twistfilter.sh > /dev/null 2>&1 &
sleep 5

# Ctlcarsystem
gnome-terminal --command "${SHELL_DIR}/15ctlreduce.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} ctlreduce`"
gnome-terminal --command ${SHELL_DIR}/15changeobstaclearea.sh
#gnome-terminal --command ${SHELL_DIR}/15update_map_route.sh
gnome-terminal --command ${SHELL_DIR}/15ctlvehicle.sh
sleep 5
gnome-terminal --command ${SHELL_DIR}/15checktlr.sh
sleep 3
gnome-terminal --command "${SHELL_DIR}/15servercheck.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} checksrv` test_drive:=${TEST_DRIVE}"
gnome-terminal --command "${SHELL_DIR}/15direct_stop.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} direct_stop`"
sleep 3
gnome-terminal --command "roslaunch drive_mode_controller drive_controller.launch"
gnome-terminal --command "roslaunch drive_stop_controller drive_stop_controller.launch"
sleep 3
gnome-terminal --command "roslaunch checksrv_checker checksrv_checker.launch"
#gnome-terminal --command ${SHELL_DIR}/15check_change_route.sh
${SHELL_DIR}/15change_config_waypoint_follower.sh            > /dev/null 2>&1 &

gnome-terminal --command ${SHELL_DIR}/16generatevehicletwist.sh
sleep 5

gnome-terminal --command "${SHELL_DIR}/16velposediffchecker.sh `python ${SHELL_DIR}/load_param.py  ${CUSTOM_YAML} vehicle_info info_path` `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} vel_pose_diff_checker`"

# 自己位置取得
${SHELL_DIR}/scan_respawn_point.sh
sleep 5

gnome-terminal --command ${SHELL_DIR}/16lostlocalization.sh

sleep 5

gnome-terminal --command "${SHELL_DIR}/RunComsDriver.sh `python ${SHELL_DIR}/load_yaml.py ${CUSTOM_YAML} plc_connector`"
sleep 2
gnome-terminal --command "roslaunch sound_box sound_box.launch"

gnome-terminal --command "roslaunch communication communication.launch"
sleep 2
gnome-terminal --command "roslaunch battery_converter battery_converter.launch"
gnome-terminal --command "roslaunch battery_checker battery_checker.launch"
gnome-terminal --command "roslaunch battery_ctlvehicle battery_ctlvehicle.launch"
sleep 2
gnome-terminal --command ${SHELL_DIR}/battery_threshold.sh
gnome-terminal --command "roslaunch battery_meter battery_meter.launch"
sleep 3
gnome-terminal --command "roslaunch shutter shutter.launch"
gnome-terminal --command "roslaunch object_detection_listener object_detection_listener.launch"
gnome-terminal --command "roslaunch traffic_lights traffic_lights.launch"

# sleep 5
# gnome-terminal --command ${SHELL_DIR}/RunMABXDriver.sh

### Simulation
### 注意!! 下記のコメントを外しただけでは車両は発進しません。以下を試してください。
###       1. ~/Autoware/param/device.lstを全て127.0.0.1に指定すること
###       2. ~/Autoware/param/monitoring_health_auto_drive.yamlに記載する監視対象ノードを/base_link_to_localizer以外全て消す事
# roslaunch waypoint_follower wf_simulator.launch initialize_source:=Rviz > /dev/null 2>&1 &
# roslaunch autoware_connector vel_pose_connect.launch sim_mode:=True > /dev/null 2>&1 &
# rosrun simulation_coms simulation_coms > /dev/null 2>&1 &
### 自動運転SW
# rostopic pub /plc_autodrive std_msgs/Int16 "data: 1" > /dev/null 2>&1 &
### 運転準備ボタン
# rostopic pub /plc_readydrive std_msgs/Int16 "data: 1" > /dev/null 2>&1 &
### 機台車両ボタン
# rostopic pub /GpioStartFlg std_msgs/Int16 "data: 1" > /dev/null 2>&1 &

gnome-terminal --command ${SHELL_DIR}/17ctl_led.sh
${SHELL_DIR}/emergency_stop.sh            > /dev/null 2>&1 &

sleep 3
##### ROSBUGの記録：運行中
gnome-terminal --command ~/Autoware/ros/logs/rosbug/RosbugRecord_AutoMode.sh
sleep 2

cp -p ${BASE_DIR}/param/monitoring_health_auto_drive.yaml ${ROS_DIR}/src/carctlsystem/packages/monitoring_health/scripts/monitoring_health.yaml
gnome-terminal --command ${SHELL_DIR}/17monitoring_health.sh
sleep 2
gnome-terminal --command ${SHELL_DIR}/17monitoring_notification.sh
sleep 2

gnome-terminal --command ${SHELL_DIR}/Respawn_back_blocker.sh
sleep 5

gnome-terminal --command ${SHELL_DIR}/17ping_node.sh
sleep 2

xdotool windowsize $(xdotool search --onlyvisible --name firefox) 100% 100%
sleep 2
xdotool windowactivate $(xdotool search --onlyvisible --name firefox)
sleep 2


logger RunSystem.sh end
exit 0
