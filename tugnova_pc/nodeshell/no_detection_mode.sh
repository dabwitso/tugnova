#!/bin/bash

rosnode kill /object_detection_listener /traffic_light_detector /shutter

read -t 5 -p "

Following nodes killed:

    Shutter
    object_detection_listener
    traffic_light_detector

"
