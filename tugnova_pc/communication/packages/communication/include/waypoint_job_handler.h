#ifndef WAYPOINT_JOB_HANDLER_H
#define WAYPOINT_JOB_HANDLER_H

#include <array>
#include <communication_msgs/RouteInfo.h>
#include <communication_msgs/RouteResponse.h>
#include <csv.h>
#include <ros/ros.h>
#include <sstream>
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

  ros::Subscriber subtask_feedback_sub;
  ros::Subscriber server_request_sub;
  ros::Subscriber battery_level_sub;

  // variables
  static const int16_t ON = 1;
  static const int16_t INIT = 0;
  const std::string DEFAULT = "";
  const std::string DEFAULT_MSG = "waiting";



  // 1/60secs = 0.017 rate. i.e, READ_CSV_REFRESH_RATE = 1/desired_refresh_time_in_secs.
  // This default of 0.017 -> refresh data from csv every minute.
  const double READ_CSV_REFRESH_RATE = 0.017;

  // number of waypoint.csv files (i.e, subtasks) to preload to get to destination
  int number_of_subtasks;

  // change this path if location of ROUTE folder changes
  const std::string ROUTE_MAP_PATH = "/home/nvidia/ROUTE/route_map.csv";

  std::string server_cmd;
  std::string battery_level;
  std::string job_status_message; // useful for visual status notification only

  std::unordered_map<std::string, std::array<std::string, 10>> route_info;

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
};

#endif
