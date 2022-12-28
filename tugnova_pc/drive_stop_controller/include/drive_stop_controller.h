#ifndef DRIVE_STOP_CONTROLLER_H
#define DRIVE_STOP_CONTROLLER_H

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>

class StopController {
  public:
  StopController();
  void run();

  private:
  ros::NodeHandle nh;

  ros::Publisher twist_cmd_pub;

  ros::Subscriber input_twist_sub;
  ros::Subscriber stop_flg_sub;
  ros::Subscriber gpio_start_drive_sub;
  ros::Subscriber start_drive_sub;

  bool isSpeedLockToZero;
  bool isStopPoint;

  static const int16_t ON = 1;
  constexpr static const double STOP_SPEED = 0.0;

  // misc functions
  void speedReducer(const geometry_msgs::TwistStamped& msg);

  // callback functions
  void inputTwistCallback(const geometry_msgs::TwistStamped& msg);
  void stopFlgCallback(const std_msgs::Int16& msg);
  void GpioStartFlgCallback(const std_msgs::Int16& msg);
  void startFlgCallback(const std_msgs::Int16& msg);
};

#endif
