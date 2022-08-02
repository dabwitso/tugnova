#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "csv.h"
#include <autoware_msgs/LaneArray.h>
#include <communication_msgs/RouteInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <string>
#include <udp_msgs/UdpSensorPacket.h>
#include <unordered_map>
#include <stdint.h>

class DriveController {
  public:
  DriveController();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher twist_cmd_pub;
  ros::Publisher waitpoint_pub;
  ros::Publisher magnet_drive_state_pub;

  ros::Subscriber plc_sensor_sub;
  ros::Subscriber drive_mode_sub;
  ros::Subscriber gpiostartflg_sub;
  ros::Subscriber direct_stop_node_sub;
  ros::Subscriber subtask_name_sub;
  ros::Subscriber reset_flags_sub;

  const std::string MAGNET_MARKER_LOOKUP_PATH = "/home/nvidia/ROUTE/magnet_count.csv";
  std::string current_route;

  bool isMagnetDrive;
  bool isMagnetStopPoint;
  bool isGpioButtonPressed;
  bool isOnMagnetStrip;

  int stop_marker_counter;
  int previous_plc_pulse_state;

  static const int ON = 1;
  static const int INIT = 0;
  static const int NOW = 0;
  static const int MAGNET_DRIVE_MODE = 2;


  // READ_CSV_REFRESH_RATE = 1/desired_refresh_time_interval_in_secs
  // default 0.017 rate -> every minute
  const double READ_CSV_REFRESH_RATE = 0.017;
  const double ZERO_SPEED = 0.0;

  std::unordered_map<std::string, int> stop_marker_lookup;

  std::string DEFAULT = "default";

  geometry_msgs::TwistStamped direct_stop_twist_cmd;

  // misc functions
  readCSV();
  speedPublisher();

  // callback functions
  void plcSensorCallback(const udp_msgs::UdpSensorPacket& msg);
  void driveModeCallback(const autoware_msgs::Lane& msg);
  void resetCallback(const std_msgs::Int16& msg);
  void subtaskInfoCallback(const communication_msgs::RouteInfo& msg);
  void inputTwistCallback(const geometry_msgs::TwistStamped& msg);
  void gpioCallback(const std_msgs::Int16& msg);
};

#endif
