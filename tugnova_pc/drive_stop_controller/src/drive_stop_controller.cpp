#include "drive_stop_controller.h"

StopController::StopController()
{
  // ros handles
  // publication
  twist_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("out_twist_cmd", 10);

  // subscription
  input_twist_sub = nh.subscribe("out_twist_raw", 10, &StopController::inputTwistCallback, this);
  stop_flg_sub = nh.subscribe("stop_drive", 1, &StopController::stopFlgCallback, this);
  gpio_start_drive_sub = nh.subscribe("GpioStartFlg", 10, &StopController::GpioStartFlgCallback, this);
  start_drive_sub = nh.subscribe("start_drive", 10, &StopController::startFlgCallback, this);

  // variable initialization
  isSpeedLockToZero = false;
  isStopPoint = false;
}

void StopController::speedReducer(const geometry_msgs::TwistStamped& msg)
{
  geometry_msgs::TwistStamped twist_msg;
  twist_msg = msg;
  double speed = twist_msg.twist.linear.x;
  // determine speed action
  if (!isSpeedLockToZero && isStopPoint) {
    ROS_INFO("Speed lock engaged");
    isSpeedLockToZero = true;
  }

  // actual speed control code
  if (isSpeedLockToZero) {
    twist_msg.twist.linear.x = STOP_SPEED;
    twist_cmd_pub.publish(twist_msg);
  } else {
    twist_cmd_pub.publish(msg);
  }
}

void StopController::inputTwistCallback(const geometry_msgs::TwistStamped& msg)
{
  speedReducer(msg);
}

void StopController::stopFlgCallback(const std_msgs::Int16& msg)
{
  if (msg.data == ON) {
    ROS_INFO("Object detection point reached");
    isStopPoint = true;
  }
}

void StopController::GpioStartFlgCallback(const std_msgs::Int16& msg)
{
  if (msg.data == ON) {
    ROS_INFO("Start button On.");
    isSpeedLockToZero = false;
    isStopPoint = false;
  }
}

void StopController::startFlgCallback(const std_msgs::Int16& msg)
{
  if (msg.data == ON) {
    ROS_INFO("Start drive On.");
    isSpeedLockToZero = false;
    isStopPoint = false;
  }
}

void StopController::run()
{
  ROS_INFO("listening...");
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_stop_controller");
  StopController stop_controller;
  stop_controller.run();

  return 0;
}
