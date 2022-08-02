#ifndef OBJECT_DETECTION_LISTENER_H
#define OBJECT_DETECTION_LISTENER_H

#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <string>
#include <climits>
//#include <stdio.h>

class ObjectListener {
  public:
  ObjectListener();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher detection_zone_pub;
  ros::Publisher stopflg_pub;
  ros::Publisher startflg_pub;

  ros::Subscriber detection_result_sub;
  ros::Subscriber detection_zone_flg_sub;

  static const int MAX_WAYPOINT_ID = INT_MAX;
  static const int ON = 1;
  static const int OFF = 0;
  static const int INIT = 0;
  static const int NOW = 0;

  /*
   SPECIAL NOTE:
   TOTAL_MESSAGE_COUNT_TO_JETSON used because jetson fails to
   detect publication when sent normally using one count.
   probably due to use of ros_bridge to ros2.
   Remove this provision when not using ros_bridge, including
   message_publish_count.
  */
  static const int TOTAL_MESSAGE_COUNT_TO_JETSON = 10;

  int message_publish_count;
  int detection_point_id;

  // lock to prevent sending activation message to camera multiple times when
  // still at same waypoint_id which would lead to infinite detection loop
  bool isStopped;

  // callback functions
  void detectionResultCallback(const std_msgs::Bool& msg);
  void detectionZoneFlgCallback(const autoware_msgs::Lane& msg)
};

#endif
