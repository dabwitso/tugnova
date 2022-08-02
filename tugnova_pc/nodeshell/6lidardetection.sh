#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

roslaunch lidar_euclidean_cluster_detect lidar_euclidean_cluster_detect.launch use_gpu:=True output_frame:=velodyne pose_estimation:=False downsample_cloud:=False points_node:=/points_no_ground leaf_size:=0.1 cluster_size_min:=20 cluster_size_max:=100000 clustering_distance:=0.75 clip_min_height:=0 clip_max_height:=2 use_vector_map:=False vectormap_frame:=map wayarea_gridmap_topic:=grid_map_wayarea wayarea_gridmap_layer:=wayarea wayarea_no_road_value:=255 remove_points_upto:=0 keep_lanes:=False keep_lane_left_distance:=5 keep_lane_right_distance:=5 cluster_merge_threshold:=1.5 use_multiple_thres:=False clustering_ranges:=[15,30,45,60] clustering_distances:=[0.5,1.1,1.6,2.1,2.6] remove_ground:=True use_diffnormals:=False publish_filtered:=False

#sleep 1000
