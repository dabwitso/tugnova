#!/bin/bash

ROUTE=$1

rostopic pub -1 /server_request std_msgs/String "data: ${ROUTE}"
echo "sent job ${ROUTE} to waypoint_job_handler"

