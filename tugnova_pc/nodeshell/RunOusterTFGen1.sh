#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
CUSTOM_YAML=/home/nvidia/Autoware/param/custom.yaml
${BASE_DIR}/common.sh
echo RunOuster
PARAM=$@

sudo systemctl restart dnsmasq

gnome-terminal --command "${BASE_DIR}/RunOusterGen1.sh ${PARAM}"

${BASE_DIR}/OusterTF.sh `python ${BASE_DIR}/load_yaml.py ${CUSTOM_YAML} ouster_lidar_tf`

#sleep 1000
