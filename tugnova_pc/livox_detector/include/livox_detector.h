#ifndef LIVOX_DETECTOR_H
#define LIVOX_DETECTOR_H

#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

class LivoxDetector {
  public:
  LivoxDetector();
  void run();

  private:
  ros::NodeHandle nh;

  ros::Publisher second_stop_pub;
  ros::Publisher livox_state_pub;

  ros::Subscriber livox_result_sub;
  ros::Subscriber second_stop_waypoint_sub;
  ros::Subscriber second_stop_flg_sub;

  static const int ON = 1;
  static const int OFF = 0;
  static const int NOW = 0;
  static const int SECOND_STOP_FLAG = 100;

  bool isSecondStop;

  std_msgs::Int16 livox_stop_state;

  // callback functions
  void livoxResultCallback(const std_msgs::Bool& msg);
  void secondStopWaypointCallback(const autoware_msgs::Lane& msg);
  void secondStopActivator(const std_msgs::Bool& msg);
};

#endif
