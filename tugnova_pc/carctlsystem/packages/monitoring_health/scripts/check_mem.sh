#!/bin/bash
# 環境変数設定
source /opt/ros/kinetic/setup.bash
source ${HOME}/catkin_ws/devel/setup.bash
source ${HOME}/Autoware/ros/devel/setup.bash

# 変数初期値代入
WARRNING=90 #警告しきい値(%)
ERROR=95    #エラーしきい値(%)
INTERVAL=60 #監視間隔(秒)
SERVICE_NAME=mem
TOPIC_NAME=/monitor_status
TYPE=carctl_msgs/monitor_status

tmpdir=/tmp
yamlfile=${tmpdir}/${SERVICE_NAME}_param.yaml
command1="rostopic pub -1 ${TOPIC_NAME} ${TYPE} -f ${yamlfile}"

function checkmem() {

  #メモリ使用率の計算
  TOTAL=$(free | grep "Mem:" | awk '{print $2}')
  AVAILABLE=$(free | grep "Mem:" | awk '{print $7}')
  USED=$(expr ${TOTAL} - ${AVAILABLE})
  VAL=$(expr 100 \* ${USED} / ${TOTAL})

  # メモリ使用率の判定処理
  if [ ${VAL} -ge ${ERROR} ]; then
    logger -p local0.err -s "ERROR memory used ${ERROR}% over ${VAL}%"
    STATUS=1
    MSG="mem_1_0"
  elif [ ${VAL} -ge ${WARRNING} ]; then
    logger -p local0.warning -s "WARNING memory used ${WARNING}% over ${VAL}%"
    STATUS=0
    MSG="WARNING memory used ${WARNING}% over ${VAL}%"
  else
    logger -p local0.info -s "INFO memory used ${VAL}%"
    STATUS=0
    MSG="INFO memory used ${VAL}%"
  fi

  echo "service_name: ${SERVICE_NAME}" >${yamlfile}
  echo "status: ${STATUS}" >>${yamlfile}
  echo "error_msg: ${MSG}" >>${yamlfile}

  ${command1}

}

# メイン処理
while true; do
  checkmem &
  2>/dev/null
  wait
  sleep ${INTERVAL}
done
