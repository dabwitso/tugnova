#!/bin/bash

BASE_DIR=/home/nvidia/Autoware
SHELL_DIR=${BASE_DIR}/ros/nodeshell
WORK_DIR=${BASE_DIR}/work
YAML_FILE=${BASE_DIR}/param/custom.yaml
TEMP_TOP=/tmp

SERVER=$(python ${SHELL_DIR}/load_param.py  ${YAML_FILE} ftp address)
USER=$(python ${SHELL_DIR}/load_param.py  ${YAML_FILE} ftp user)
PASS=$(python ${SHELL_DIR}/load_param.py  ${YAML_FILE} ftp password)

ROUTE_NAME=${1}

cd ${SHELL_DIR}
./zip.sh ${ROUTE_NAME} ${WORK_DIR}
ZIP_FILE=$(<${TEMP_TOP}/ZIP_FILENAME)
mv ${ZIP_FILE} ${WORK_DIR}/ZIP

ftp -n <<EOF
open $SERVER
user $USER $PASS
bin
lcd ${WORK_DIR}/ZIP
put ${ZIP_FILE#"${TEMP_TOP}/"}
bye
EOF

