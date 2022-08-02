#ifndef WAYPOINT_SELECTOR_H
#define WAYPOINT_SELECTOR_H

#include "csv.h"
#include <algorithm>
#include <climits>
#include <communication_msgs/RouteInfo.h>
#include <communication_msgs/RouteResponse.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std::string>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <unordered_map>

class WaypointSelector {
  public:
  WaypointSelector();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher subtask_update_pub;

  ros::Subscriber subtask_cmd_sub;
  ros::Subscriber waypoint_sub;

  // READ_CSV_ROUTE_DIR_REFRESH_RATE = 1/desired_refresh_time_in_secs;
  // default 0.017 -> refresh every minute
  const double READ_CSV_ROUTE_DIR_REFRESH_RATE = 0.017;
  // TOLERANCE=1,2,....10. This adjust how strict position comparison should be for route change calculations.
  const double TOLERANCE = 1;

  const int MAX_WAYPOINT_ID = INT_MAX;

  const std::string DEFAULT = "";
  std::string subtask_route_id;

  // paths
  const std::string CSV_ROUTE_PATH = "/home/nvidia/ROUTE/waypoint_dir.csv";
  const std::string IN_SHELL_SCRIRT_PATH = "/home/nvidia/Autoware/ros/nodeshell/12waypointloader_static.sh";
  const std::string OUT_SHELL_SCRIRT_PATH = "/home/nvidia/Autoware/ros/nodeshell/12waypointloader.sh";
  const std::string RESPAWN_SHELL_SCRIRT_PATH = "/home/nvidia/Autoware/ros/nodeshell/execute_respawn_node.sh";

  std::unordered_map<std::string, std::string> route_dictionary;

  // double x_coordinate, y_coordinate;
  int last_waypoint;
  bool is_route_completed;

  // misc functions
  void route_append(std::string& route_name);
  void read_csv(std::string& file_types, std::string& dir_location);
  void waypoint_changer(std::string& route);
  void change_case(std::string& strtxt)

  // callback functions
  void closestWaypointCallback(const std_msgs::Int32& msg);
  void subtaskCallback(const communication_msgs::RouteInfo& message);
};

#endif
