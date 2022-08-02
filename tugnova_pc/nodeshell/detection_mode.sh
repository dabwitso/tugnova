#!/bin/bash

gnome-terminal --command "roslaunch shutter shutter.launch"

gnome-terminal --command "roslaunch object_detection_listener object_detection_listener.launch"

gnome-terminal --command "roslaunch traffic_lights traffic_lights.launch"

read -t 5 -p "

Following nodes enables:

     traffic_light_detector
     shutter
     object_detection_listener

"
