#ifndef DRIVE_STOP_CONTROLLER_H
#define DRIVE_STOP_CONTROLLER_H

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <stdint.h>

class StopController {
  public:
  StopController();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher twist_cmd_pub;

  ros::Subscriber input_twist_sub;
  ros::Subscriber shutter_stopflg_sub;
  ros::Subscriber stop_flg_sub;
  ros::Subscriber gpio_start_drive_sub;
  ros::Subscriber start_drive_sub;

  bool isShutterStopPoint;
  bool isSpeedLockToZero;
  bool isStopPoint;

  int stop_duration_counter;
  // decelerate speed by these levels from 100% to 0%
  // for smooth stop transition at shutter stop points
  const double LEVEL_25 = 0.25;
  const double LEVEL_35 = 0.35;
  const double LEVEL_50 = 0.50;
  const double LEVEL_75 = 0.75;
  const double STOP_SPEED = 0.0;

  // determines how soon car stops smoothly at shutter stop point
  // default after 200 cycles i.e 100 cycles per sec
  static const int STOP_SMOOTHLY_DURATION = 200;
  static const int STEP_75 = (int)(STOP_SMOOTHLY_DURATION * LEVEL_75);
  static const int STEP_50 = (int)(STOP_SMOOTHLY_DURATION * LEVEL_50);
  static const int STEP_25 = (int)(STOP_SMOOTHLY_DURATION * LEVEL_25);

  static const int16_t ON = 1;
  static const int16_t INIT = 0;

  // misc functions
  void speedReducer(const geometry_msgs::TwistStamped& msg);

  // callback functions
  void inputTwistCallback(const geometry_msgs::TwistStamped& msg);
  void shutterStopFlgCallback(const std_msgs::Int16& msg);
  void stopFlgCallback(const std_msgs::Int16& msg);
  void GpioStartFlgCallback(const std_msgs::Int16& msg);
  void startFlgCallback(const std_msgs::Int16& msg);
};

#endif
