#ifndef SHUTTER_CONTROLLER_H
#define SHUTTER_CONTROLLER_H

#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <stdint.h>

struct BluetoothFlags {
  static const int CONNECTION_SUCCESS = 1;
  static const int CONNECTION_RETRY = 2;
  static const int CONNECTION_ERROR = 3;
  static const int CONNECTION_TIMEOUT = 4;
};

class ShutterController {
  public:
  ShutterController();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher bluetooth_connect_req_pub;
  ros::Publisher passed_through_shutter_pub;
  ros::Publisher shutter_stoppoint_stopflg_pub;
  ros::Publisher shutter_resume_drive_pub;
  ros::Publisher shutter_error_pub;

  ros::Subscriber shutter_response_sub;
  ros::Subscriber bluetooth_connect_sub;
  ros::Subscriber shutter_waypoint_flg_sub;
  ros::Subscriber clear_waitpoint_sub;
  ros::Subscriber gpiostartflg_sub;

  BluetoothFlags connection_state;

  static const int16_t ON = 1;
  static const int16_t OFF = 0;
  static const int16_t INIT = 0;
  static const int NOW = 0;

  int shutterStopCount;
  int bluetooth_flg;
  int error;

  bool isShutterOpen;
  bool isWaitingBTConnection;

  // misc functions
  void shutterStopHandler();

  // callback functions
  void shutterFlgsCallback(const autoware_msgs::Lane& msg);
  void shutterResponseCallback(const std_msgs::String& msg);
  void clearWaitpointCallback(const std_msgs::String& msg);
  void gpioStartFlgCallback(const std_msgs::Int16& msg);
  void shutterBluetoothCallback(const std_msgs::Int8& msg)
};

#endif
