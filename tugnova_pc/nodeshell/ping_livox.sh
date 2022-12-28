#!/bin/bash

PID=$$
LIVOX_IP=$1

echo "Publishing script's PID: ${PID}"
rostopic pub -1 /ping_script_pid std_msgs/Int32 "data: ${PID}"

while [ 1 ];
do
ping -W 1 -c 2 ${LIVOX_IP} >&/dev/null

if [ $? -ne 0 ];
then
  echo "ERROR: Livox on ip ${LIVOX_IP} Not Reacheable!"
  rostopic pub -1 /livox_ping_status std_msgs/Int16 "data: 1"
else
  echo "STATUS: Livox on ip ${LIVOX_IP} Connected!"
  rostopic pub -1 /livox_ping_status std_msgs/Int16 "data: 0"
fi
done
