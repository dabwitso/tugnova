#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "csv.h"
#include <autoware_msgs/LaneArray.h>
#include <chrono>
#include <climits>
#include <communication_msgs/RouteInfo.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <stdint.h>
#include <string>
#include <udp_msgs/UdpSensorPacket.h>
#include <unordered_map>

typedef std::chrono::steady_clock chrono_clock;
using std::unordered_map;

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
  ros::Publisher magnet_stop_plc_pub;
  ros::Publisher stop_drive_pub;

  ros::Subscriber plc_sensor_sub;
  ros::Subscriber drive_mode_sub;
  ros::Subscriber gpiostartflg_sub;
  ros::Subscriber direct_stop_node_sub;
  ros::Subscriber subtask_name_sub;
  ros::Subscriber reset_flags_sub;
  ros::Subscriber cart_id_sub;
  ros::Subscriber csv_refresh_clock_sub;

  const std::string MAGNET_MARKER_LOOKUP_PATH = "/home/nvidia/Autoware/ROUTE/magnet_count.csv";
  const std::string MAGNET_POSITION_CSV_DIR = "/home/nvidia/Autoware/ROUTE/magnet_count.csv";
  std::string current_route;
  std::string cart_id;

  bool isMagnetDrive;
  bool isMagnetStopPoint;
  bool isGpioButtonPressed;
  bool isOnMagnetStrip;
  bool isMagnetDataRecorded;
  bool isStartTimeRecorded;
  bool isCsvRefresh;

  int stop_marker_counter;
  int previous_plc_pulse_state;
  int dict_stop_point;

  std::chrono::steady_clock::time_point start_time;
  // std::chrono::steady_clock::time_point current_time;

  static const int ON = 1;
  static const int INIT = 0;
  static const int NOW = 0;
  static const int MAGNET_DRIVE_MODE = 2;
  static const int MANUAL_DRIVE = 0;

  const double ZERO_SPEED = 0.0;

  // in milliseconds obtained from file at MAGNET_MARKER_LOOKUP_PATH
  size_t cart_stop_time;

  // Multidimensional map of following structure example
  /*
    magnet_count_lookup = { route_1:
                                   stop_position:{
                                   "cart1": 2000(ms),
                                   "cart2": 1000(ms),
                                   "cart3": 1200(ms),
                                   }
                          }
   */
  unordered_map<std::string, unordered_map<int, unordered_map<std::string, size_t>>> magnet_count_lookup;

  std::string DEFAULT = "default";
  std::string DEFAULT_CART_ID = "cart1";

  // misc functions
  void readCSV();
  void stopPublisher();

  // callback functions
  void plcSensorCallback(const udp_msgs::UdpSensorPacket& msg);
  void driveModeCallback(const autoware_msgs::Lane& msg);
  void resetCallback(const std_msgs::Int16& msg);
  void subtaskInfoCallback(const communication_msgs::RouteInfo& msg);
  void gpioCallback(const std_msgs::Int16& msg);
  void cartidCallback(const std_msgs::String& msg);
  void csvRefreshTriggerCallback(const std_msgs::Int8& msg);
};

#endif
