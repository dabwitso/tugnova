#!/bin/bash
API_DIR=/home/nvidia/Autoware/api
BASE_DIR=/home/nvidia/Autoware/ros
${BASE_DIR}/nodeshell/common.sh
# ./common.sh

##### シスログ・latestの記録
gnome-terminal --command "/home/nvidia/Autoware/ros/logs/systemlog/Create_Tar_LogFile.sh"

logger ExitAutoware.sh start

xdotool windowactivate --sync $(xdotool search --name "Runtime Manager") key --clearmodifiers --delay 100 alt+F4

#Escape ndt_matching log###
${BASE_DIR}/shutdown

wait
###########################


#Rosnode all kill##########
rosnode kill -a
###########################


#Roscore kill##############
killall roscore
###########################

#Runtimemanger GUI kill####
PID=$(ps -ef | grep runtime_manager_dialog.py | awk '{print $2}')

sudo kill $PID
###########################

#plc_connector.launch kill####
PID=$(ps -ef | grep plc_connector.launch | awk '{print $2}')

sudo kill -9 $PID
###########################

${BASE_DIR}/nodeshell/LED_nostate.sh

###########################

ps -ef |grep location_monitor.sh |grep -v grep  |awk '{print $2}' |xargs -I[] kill []
ps -ef |grep twist_monitor.sh |grep -v grep  |awk '{print $2}' |xargs -I[] kill []
ps -ef |grep "/twist_cmd" |grep -v grep  |awk '{print $2}' |xargs -I[] kill []
ps -ef |grep "/safety_waypoints/waypoints\[0\]/wpc/waypoint_id"  |grep -v grep  |awk '{print $2}' |xargs -I[] kill []

echo "non-working,init,api_1_9,init" > ${API_DIR}/VEHICLE_STATUS
echo "non-working,0" > ${API_DIR}/VEHICLE_LOCATION

logger ExitAutoware.sh end
#ps auxw | grep plc_connector.launch |grep -v grep |awk '{print "kill -9", $2}' | sh
