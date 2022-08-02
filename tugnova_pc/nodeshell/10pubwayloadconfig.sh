#!/bin/bash
BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh
rostopic pub /config/waypoint_replanner autoware_config_msgs/ConfigWaypointReplanner '{multi_lane_csv: $1,replanning_mode: False,velocity_max: 20.0,velocity_min: 4.0,accel_limit: 0.980000019073,decel_limit: 0.980000019073,radius_thresh: 20.0,radius_min: 6.0,resample_mode: True,resample_interval: 1.0,velocity_offset: 4,end_point_offset: 5,braking_distance: 5,replan_curve_mode: False,replan_endpoint_mode: True,overwrite_vmax_mode:,False,realtime_tuning_mode: False}'

#sleep 1000

