#!/bin/bash

cd /home/nvidia/Autoware/ros

source /opt/ros/kinetic/setup.bash


read -p "This script will build all autoware packages. 
Press [Ctrl + C] to quit, [Enter] to continue"


echo "

Now compiling all message packages first.... 

"
sleep 4

catkin_make --pkg communication_msgs udp_msgs carctl_msgs autoware_msgs autoware_can_msgs autoware_system_msgs dbw_mkz_msgs autoware_config_msgs 

echo "

Now compiling all packages ...

"
sleep 4

catkin_make -DCATKIN_BlACKLIST_PACKAGES="communication_msgs;udp_msgs;carctl_msgs;autoware_msgs;autoware_can_msgs;autoware_system_msgs;dbw_mkz_msgs;autoware_config_msgs" 


echo "

Compiling completed. 

Note: If compiling terminated due to errors, fix packages causing errors and run:
	

     cd /home/nvidia/Autoware/ros && source /opt/ros/kinetic/setup.bash && catkin_make

"
