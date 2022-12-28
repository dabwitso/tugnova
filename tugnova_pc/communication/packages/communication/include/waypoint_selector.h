#ifndef WAYPOINT_SELECTOR_H
#define WAYPOINT_SELECTOR_H

#include "csv.h"
#include <algorithm>
#include <climits>
#include <communication_msgs/RouteInfo.h>
#include <communication_msgs/RouteResponse.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <stdio.h>
#include <unordered_map>
#include <array>

class WaypointSelector {
  public:
  WaypointSelector();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher subtask_update_pub;
  ros::Publisher hotspot_error_pub;
  ros::Publisher closest_waypoint_pub;
  ros::Publisher last_waypoint_pub;

  ros::Subscriber subtask_cmd_sub;
  ros::Subscriber waypoint_sub;
  ros::Subscriber csv_refresh_clock_sub;
  ros::Subscriber hotspot_mode_sub;
  ros::Subscriber blue_line_sub;
  ros::Subscriber hotspot_script_sub;

  int last_waypoint;
  bool isRouteCompleted;
  bool isCsvRefresh;
  bool isFileMissing;
  bool isHotspotSet;
  bool isSkipBluelineCheck;
  bool isCheckClosestWaypoint;

  // TOLERANCE=1,2,....10. This adjust how strict position comparison should be for route change calculations.
  const double TOLERANCE = 1;

  static const int MAX_WAYPOINT_ID = INT_MAX;
  static const int MISSING_FILE_ERROR_FLG = 2;
  // offset when job_complete signal is triggered to freedom.
  // last_waypoint = last_waypoint_id_in_csv_filg - GOAL_WAYPOINT_OFFSET
  static const int GOAL_WAYPOINT_OFFSET = 5;
  static const int ON = 1;
  static const int OFF = 0;

  const std::string DEFAULT = "";
  const std::string ROUTE_ACCEPTED = "Accepted";
  const std::string ROUTE_REJECTED = "Rejected";
  const std::string WAITING_FOR_TASK = "Available";
  const std::string TASK_COMPLETE = "Completed";

  std::string subtask_route_id;
  std::string hotspot_mode;

  std::array<std::string, 2> BLUE_LINE_NG;
  std::array<std::string, 5> IGNORE_LIST;

  // paths
  const std::string CSV_ROUTE_PATH = "/home/nvidia/Autoware/ROUTE/waypoint_dir.csv";
  const std::string HOTSPOT_PATH = "/home/nvidia/Autoware/ROUTE/hotspot.csv";
  const std::string IN_SHELL_SCRIRT_PATH = "/home/nvidia/Autoware/ros/nodeshell/12waypointloader_static.sh";
  const std::string OUT_SHELL_SCRIRT_PATH = "/home/nvidia/Autoware/ros/nodeshell/12waypointloader.sh";
  const std::string RESPAWN_SHELL_SCRIRT_PATH = "/home/nvidia/Autoware/ros/nodeshell/execute_respawn_node.sh";
  const std::string SPAWN_HOTSPOT_SCRIPT = "/home/nvidia/Autoware/ros/nodeshell/set_hotspot.sh";

  std::string hotspot_dir;
  std::unordered_map<std::string, std::string> route_dictionary;


  // misc functions
  void route_append(std::string& route_name);
  void read_csv(std::string&& file_types, std::string& dir_location);
  void waypoint_changer(std::string& route);
  void change_case(std::string& strtxt);
  bool check_hotspot(std::string& route_id);

  // callback functions
  void closestWaypointCallback(const std_msgs::Int32& msg);
  void subtaskCallback(const communication_msgs::RouteInfo& msg);
  void csvRefreshTriggerCallback(const std_msgs::Int8& msg);
  void hotspotModeCallback(const std_msgs::String& msg);
  void isBluelineCallback(const std_msgs::String& msg);
  void hotspotScriptCallback(const std_msgs::Int16& msg);
};

#endif
