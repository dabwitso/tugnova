#!/bin/bash

SOURCE_DIR=/home/nvidia/gitlab/routes/
DESTINATION_DIR=/home/nvidia/Autoware/ROUTE/


echo "
cleaning folder ${DESTINATION_DIR}...

"
rm ${DESTINATION_DIR}*.csv

echo "
loading route files from ${SOURCE_DIR} into ${DESTINATION_DIR} ...

"
cp ${SOURCE_DIR}*.csv ${DESTINATION_DIR}

echo "Completed"
