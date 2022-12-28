#ifndef WAYPOINT_JOB_HANDLER_H
#define WAYPOINT_JOB_HANDLER_H

#include <array>
#include <algorithm>
#include <communication_msgs/RouteInfo.h>
#include <communication_msgs/RouteResponse.h>
#include <csv.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string>
#include <unordered_map>

class JobScheduler {
  public:
  JobScheduler();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher route_cmd_pub;
  ros::Publisher reply_server_pub;
  ros::Publisher keep_tugnova_stopped_pub;

  ros::Subscriber subtask_feedback_sub;
  ros::Subscriber server_request_sub;
  ros::Subscriber battery_level_sub;
  ros::Subscriber csv_refresh_clock_sub;

  // variables
  bool isCsvRefresh;

  static const int16_t ON = 1;
  static const int16_t INIT = 0;
  static const int STATUS_SEND_COUNT = 3;

  const std::string DEFAULT = "";
  const std::string DEFAULT_MSG = "waiting";
  const std::string STARTING_ROUTE = "route1";

  // number of waypoint.csv files (i.e, subtasks) to preload to get to destination
  int number_of_subtasks;

  // change this path if location of ROUTE folder changes
  const std::string ROUTE_MAP_PATH = "/home/nvidia/Autoware/ROUTE/route_map.csv";

  std::string server_cmd;
  std::string battery_level;
  std::string job_status_message; // useful for visual status notification only
  std::string param_val;

  std::unordered_map<std::string, std::array<std::string, 10>> route_info;
  std::array<std::string,5> tugnova_states;

  // misc functions

  void split_string(const std::string& s, char delimiter, std::string dict_key);
  void route_update(std::string previous_state);
  void change_case(std::string& txt);
  void assign_job();
  void read_csv();

  // callback functions

  void routeCmdCallback(const std_msgs::String& msg);
  void subtaskUpdateCallback(const communication_msgs::RouteResponse& message);
  void batteryLevelCallback(const std_msgs::String& msg);
  void csvRefreshTriggerCallback(const std_msgs::Int8& msg);
};

#endif
