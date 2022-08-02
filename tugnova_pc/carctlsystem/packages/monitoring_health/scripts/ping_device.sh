#!/bin/bash
# 環境変数設定
source /opt/ros/kinetic/setup.bash
source ${HOME}/catkin_ws/devel/setup.bash
source ${HOME}/Autoware/ros/devel/setup.bash

# 変数初期値代入
param_dir="${HOME}/Autoware/param"
devicelist="${param_dir}/device.lst"

yaml=${HOME}/Autoware/ros/src/carctlsystem/packages/monitoring_health/scripts/monitoring_health.yaml
modeline=$(grep -E '^\s+mode\s*:' ${yaml})
mode=$(cut -d':' -f 2 <<<${modeline// /})

dic_dir="${HOME}/Autoware/ros/renkei/DICTIONARY"
if [ "${mode}" == "route-creation" ]; then
  dic_dir="${HOME}/Autoware/work/DICTIONARY"
fi
diclist="${dic_dir}/DICTIONARY"

INTERVAL=1
SERVICE_NAME=device
TOPIC_NAME=/monitor_status
TYPE=carctl_msgs/monitor_status

tmpdir=/tmp
yamlfile=${tmpdir}/${SERVICE_NAME}_param.yaml
command1="rostopic pub -1 ${TOPIC_NAME} ${TYPE} -f ${yamlfile}"

function pubdev() {
  STATUS=${1}
  messageId=${2}
  echo "service_name: ${SERVICE_NAME}" >${yamlfile}
  echo "status: ${STATUS}" >>${yamlfile}
  echo "error_msg: ${messageId}" >>${yamlfile}
  echo "DEBUG: pubdev: STATUS:${STATUS},messageId:${messageId}"

  ${command1}
  unset STATUS
  unset messageId
}

function pingdev() {

  VAL=0
  LIST=""

  for line in `cat ${devicelist}`; do
    DEV=$(echo ${line} | awk -F: '{print $1}')
    IP=$(echo ${line} | awk -F: '{print $2}')
    ping -W 1 -c 3 ${IP} >/dev/null
    RESULT=$?
    if [ ${RESULT} -ne 0 ]; then
      VAL=$(expr ${VAL} + 1)
      LIST="${LIST} ${DEV}"
      STATUS=1
      MSG_=$(grep "device_1_" ${diclist} | grep ${DEV})
      RESULT=$?
      if [ ${RESULT} -eq 0 ]; then
        MSG=$(echo ${MSG_} | awk -F\" '{print $4}')
      else
        echo "ERROR Device not found in the DICTIONARY"
        MSG="device_1_0"
      fi
      pubdev ${STATUS} ${MSG}
    fi

  done

  # デバイス監視の判定処理
  if [ ${VAL} -gt 0 ]; then
    logger -p local0.err -s "ERROR device down device name ${LIST}"
    # STATUS=1
    # MSG="ERROR device down device name ${LIST}"
  else
    logger -p local0.info -s "INFO device status ok"
    STATUS=0
    MSG="device_0_1"
    pubdev ${STATUS} ${MSG}
  fi

  #echo "service_name: ${SERVICE_NAME}" > ${yamlfile}
  #echo "status: ${STATUS}" >> ${yamlfile}
  #echo "error_msg: ${MSG}" >> ${yamlfile}

  #${command1}

}

# メイン処理
while true; do
  pingdev &
  2>/dev/null
  wait
  sleep ${INTERVAL}
done
