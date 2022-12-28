#ifndef SHUTTER_CONTROLLER_H
#define SHUTTER_CONTROLLER_H

#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>


class ShutterController {
  public:
  ShutterController();
  void run();

  private:
  ros::NodeHandle nh;

  ros::Publisher shutter_stop_pub;

  ros::Subscriber shutter_waypoint_flg_sub;


  static const int ON = 1;
  static const int INIT = 0;
  static const int NOW = 0;

  int shutterStopCount;

  // misc functions
  void shutterStopHandler();

  // callback functions
  void shutterFlgsCallback(const autoware_msgs::Lane& msg);
};

#endif
