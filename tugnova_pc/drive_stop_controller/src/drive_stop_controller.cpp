#include "drive_stop_controller.h"

StopController::StopController()
    : private_nh("~")
{
  // ros handles
  // publication
  twist_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("out_twist_cmd", 10);

  // subscription
  input_twist_sub = nh.subscribe("out_twist_ctrl", 10, &StopController::inputTwistCallback, this);
  shutter_stopflg_sub = nh.subscribe("shutter_stop_flag", 10, &StopController::shutterStopFlgCallback, this);
  stop_flg_sub = nh.subscribe("stop_drive", 1, &StopController::stopFlgCallback, this);
  gpio_start_drive_sub = nh.subscribe("GpioStartFlg", 10, &StopController::GpioStartFlgCallback, this);
  start_drive_sub = nh.subscribe("start_drive", 10, &StopController::startFlgCallback, this);

  // variable initialization
  isShutterStopPoint = false;
  isSpeedLockToZero = false;
  isStopPoint = false;

  stop_duration_counter = INIT;
}

void StopController::speedReducer(const geometry_msgs::TwistStamped& msg)
{
  geometry_msgs::TwistStamped twist_msg;
  twist_msg = msg;
  double speed = twist_msg.twist.linear.x;
  // determine speed action
  if (!isSpeedLockToZero) {
    if (isShutterStopPoint) {
      if (stop_duration_counter < STOP_SMOOTHLY_DURATION) {
        if (stop_duration_counter <= STEP_25) {
          twist_msg.twist.linear.x = speed * LEVEL_75;
        } else if (stop_duration_counter > STEP_25 && stop_duration_counter <= STEP_50) {
          twist_msg.twist.linear.x = speed * LEVEL_50;
        } else if (stop_duration_counter > STEP_50 && stop_duration_counter <= STEP_75) {
          twist_msg.twist.linear.x = speed * LEVEL_35;
        } else if (stop_duration_counter > STEP_75) {
          twist_msg.twist.linear.x = speed * LEVEL_25;
        }
        twist_cmd_pub.publish(twist_msg);
        stop_duration_counter++;
      } else {
        isSpeedLockToZero = true;
      }
    } else if (isStopPoint) {
      if (twist_msg.twist.linear.x == 0)
        isSpeedLockToZero = true;
    }
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

void StopController::shutterStopFlgCallback(const std_msgs::Int16& msg)
{
  if (msg.data == ON) {
    ROS_INFO("shutter stop flag recieved");
    isShutterStopPoint = true;
  }
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
    isShutterStopPoint = false;
    isSpeedLockToZero = false;
    stop_duration_counter = INIT;
    isStopPoint = false;
  }
}

void StopController::startFlgCallback(const std_msgs::Int16& msg)
{
  if (msg.data == ON) {
    ROS_INFO("Start drive On.");
    isShutterStopPoint = false;
    isSpeedLockToZero = false;
    stop_duration_counter = INIT;
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
