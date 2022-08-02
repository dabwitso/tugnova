#!/bin/bash
BASE_DIR=/home/nvidia/Autoware
ROS_DIR=${BASE_DIR}/ros
SHELL_DIR=${ROS_DIR}/nodeshell
RTMGR_DIR=${ROS_DIR}/src/util/packages/runtime_manager/scripts
PARAM_YAML=${RTMGR_DIR}/param.yaml

${SHELL_DIR}/common.sh

LEAF_SIZE=$(python ${SHELL_DIR}/load_param.py  ${PARAM_YAML} voxel_grid_filter voxel_leaf_size)
MEASUREMENT_RANGE=$(python ${SHELL_DIR}/load_param.py  ${PARAM_YAML} voxel_grid_filter measurement_range)

rostopic pub /config/voxel_grid_filter autoware_config_msgs/ConfigVoxelGridFilter "voxel_leaf_size: $LEAF_SIZE
measurement_range: $MEASUREMENT_RANGE"


#sleep 1000
