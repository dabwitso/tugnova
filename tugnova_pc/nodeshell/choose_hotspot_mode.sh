#!/bin/bash

# MODE is either A or B, representing hotspot alternative per route file
MODE=$1

rostopic pub -1 /hotspot_mode std_msgs/String "data: '$MODE'"
