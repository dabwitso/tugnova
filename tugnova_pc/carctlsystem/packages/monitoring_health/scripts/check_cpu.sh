#!/bin/bash
# 環境変数設定
source /opt/ros/kinetic/setup.bash
source ${HOME}/catkin_ws/devel/setup.bash
source ${HOME}/Autoware/ros/devel/setup.bash

# 変数初期値代入
WARRNING=80 #警告しきい値(%)
ERROR=90    #エラーしきい値(%)
INTERVAL=60 #監視間隔(秒)
SERVICE_NAME=cpu
TOPIC_NAME=/monitor_status
TYPE=carctl_msgs/monitor_status

tmpdir=/tmp
yamlfile=${tmpdir}/${SERVICE_NAME}_param.yaml
command1="rostopic pub -1 ${TOPIC_NAME} ${TYPE} -f ${yamlfile}"

function checkcpu() {

  # CPUのアイドル値を取得
  VMSTAT=$(vmstat ${INTERVAL} 2 2>/dev/null | tail -1)
  # US=`echo ${VMSTAT} | awk '{print $13}'`
  # SY=`echo ${VMSTAT} | awk '{print $14}'`
  # TOTAL=`expr ${US} + ${SY}`
  ID=$(echo ${VMSTAT} | awk '{print $15}')

  # CPUの使用率を計算
  # VAL=`expr 100 - ${TOTAL}`
  VAL=$(expr 100 - ${ID})

  # CPU使用率の判定処理
  if [ ${VAL} -ge ${ERROR} ]; then
    logger -p local0.err -s "ERROR cpu used ${ERROR}% over ${VAL}%"
    STATUS=1
    MSG="cpu_1_0"
  elif [ ${VAL} -ge ${WARRNING} ]; then
    logger -p local0.warning -s "WARNING cpu used ${WARNING}% over ${VAL}%"
    STATUS=0
    MSG="WARNING cpu used ${WARNING}% over ${VAL}%"
  else
    logger -p local0.info -s "INFO cpu used ${VAL}%"
    STATUS=0
    MSG="INFO cpu used ${VAL}%"
  fi

  echo "service_name: ${SERVICE_NAME}" >${yamlfile}
  echo "status: ${STATUS}" >>${yamlfile}
  echo "error_msg: ${MSG}" >>${yamlfile}

  ${command1}

}

# メイン処理
while true; do
  checkcpu &
  2>/dev/null
  wait
  # sleep ${INTERVAL}
done
