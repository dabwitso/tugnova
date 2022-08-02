#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
roslaunch vision_darknet_detect vision_yolo3_detect.launch score_threshold:=0.5 nms_threshold:=0.45 image_src:=/zed/rgb/image_raw_color network_definition_file:=/home/nvidia/Autoware/ros/src/computing/perception/detection/vision_detector/packages/vision_darknet_detect/darknet/cfg/yolov3.cfg pretrained_model_file:=/home/nvidia/Autoware/ros/src/computing/perception/detection/vision_detector/packages/vision_darknet_detect/darknet/data/yolov3.weights names_file:=/home/nvidia/Autoware/ros/src/computing/perception/detection/vision_detector/packages/vision_darknet_detect/darknet/cfg/coco.names gpu_device_id:=0 camera_id:=

#sleep 1000
