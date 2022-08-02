#!/bin/bash

BASE_DIR=/home/nvidia/Autoware
VEHICLE_STATUS=${BASE_DIR}/api/VEHICLE_STATUS
LASTPOSE_DIR=${BASE_DIR}/LastPose
NDT_MATCHING_LOG=${LASTPOSE_DIR}/ndt_matching_log.csv

LANE_ID_FILE=${BASE_DIR}/current_lane_id
LANE_INFO=$(<${LANE_ID_FILE})
courseid=$(cut -d'_' -f 1 <<<${LANE_INFO})
ZIP_DIR_HOTSPOTS=${BASE_DIR}/work/${courseid}/HOTSPOTS


if [ ! -f ${VEHICLE_STATUS} ]; then
  echo "[error] not found VEHICLE_STATUS."
  exit 1
fi

BUFFER=$(<${VEHICLE_STATUS})
IS_PERMIT=true
for STATUS in ${BUFFER//,/ }; do
  if [ ${STATUS} = "non-working" ]; then
    IS_PERMIT=false
    break
  fi
  break
done

if ${IS_PERMIT}; then
  echo "[error] please stop Autoware."
  exit 1
fi


if [ ! -f ${NDT_MATCHING_LOG} ]; then
  echo "[error] not found ndt_matching_log.csv"
  exit 1
fi

tgtArray=$(cat ${NDT_MATCHING_LOG} | awk 'END{print $0}')
IFS=','
set -- $tgtArray
X=${5}
Y=${6}

COUNT=`ls -1 ${ZIP_DIR_HOTSPOTS}/HOTSPOT_*.csv 2>/dev/null | wc -l`
let COUNT++

NEW_FILE=${ZIP_DIR_HOTSPOTS}/HOTSPOT_${COUNT}_${X}_${Y}.csv
cp -p ${NDT_MATCHING_LOG} ${NEW_FILE}

echo "[success] create new HOTSPOT."
echo "==> ${NEW_FILE}"
