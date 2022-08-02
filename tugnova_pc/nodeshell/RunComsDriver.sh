#!/bin/bash

BASE_DIR=/home/nvidia/Autoware/ros/nodeshell
${BASE_DIR}/common.sh

# roslaunch udp_connector plc_connector.launch coms:=yukuri port_receive:=5001 port_send:=5002 destination:=192.168.1.78 use_low_pass_filter:=True use_median_filter:=True tire_radius:=0.92 tire_distance:=0.25 encoder_pulse_resolution:=1800
# roslaunch udp_connector plc_connector.launch coms:=yukuri port_receive:=5001 port_send:=5002 destination:=192.168.0.10 use_low_pass_filter:=True use_median_filter:=True tire_radius:=0.92 tire_distance:=0.25 encoder_pulse_resolution:=1800
roslaunch udp_connector plc_connector.launch $@