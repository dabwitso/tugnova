#include "waypoint_selector.h"

WaypointSelector::WaypointSelector()
    : private_nh("~")
{
  // ros handles
  // publication
  subtask_update_pub = nh.advertise<communication_msgs::RouteResponse>("route_request_fbk", 10);
  hotspot_error_pub = nh.advertise<std_msgs::Int16>("hotspot_error", 10);

  // debug only
  closest_waypoint_pub = nh.advertise<std_msgs::Int32>("waypoint_selector_closest", 10);
  last_waypoint_pub = nh.advertise<std_msgs::Int32>("waypoint_selector_last", 10);
  // end of debug code

  // subscription
  subtask_cmd_sub = nh.subscribe("route_cmds", 1, &WaypointSelector::subtaskCallback, this);
  waypoint_sub = nh.subscribe("closest_waypoint", 1, &WaypointSelector::closestWaypointCallback, this);
  csv_refresh_clock_sub = nh.subscribe("csv_refresh", 1, &WaypointSelector::csvRefreshTriggerCallback, this);
  hotspot_mode_sub = nh.subscribe("hotspot_mode", 1, &WaypointSelector::hotspotModeCallback, this);
  blue_line_sub = nh.subscribe("send_error_status", 1, &WaypointSelector::isBluelineCallback, this);
  hotspot_script_sub = nh.subscribe("hotspot_error", 1, &WaypointSelector::hotspotScriptCallback, this);

  // variable initialization
  last_waypoint = MAX_WAYPOINT_ID;
  subtask_route_id = DEFAULT;
  isRouteCompleted = false;
  isCsvRefresh = true;
  isHotspotSet = false;
  isFileMissing = false;
  isCheckClosestWaypoint = false;

  hotspot_dir = "test";
  hotspot_mode = "A";
  isSkipBluelineCheck = false;
  BLUE_LINE_NG = { "lost_localization_1_0", "lost_localization_1_2" };
  IGNORE_LIST = { "server_0_0", "server_0_1", "server_2_0", "server_0_2", "server_2_1" };
}
void WaypointSelector::hotspotScriptCallback(const std_msgs::Int16& msg)
{
  if (msg.data == MISSING_FILE_ERROR_FLG) {
    isSkipBluelineCheck = true;
  }
}

void WaypointSelector::hotspotModeCallback(const std_msgs::String& msg)
{
  hotspot_mode = msg.data;
}

bool WaypointSelector::check_hotspot(std::string& route)
{
  try {
    io::CSVReader<4> in(HOTSPOT_PATH);
    in.read_header(io::ignore_extra_column, "route_id", "hotspot_A", "hotspot_B", "hotspot_C");
    std::string route_id, hotspot_A, hotspot_B, hotspot_C;
    ROS_INFO("Reading hotspot.csv");
    while (in.read_row(route_id, hotspot_A, hotspot_B, hotspot_C)) {
      // find if route_id is a hotspot zone
      if (route == route_id) {
        // check which hotspot mode to load
        if (hotspot_mode == "A") {
          hotspot_dir = hotspot_A;
        } else {
          std::string temp = hotspot_mode == "B" ? hotspot_B : hotspot_C;
          change_case(temp);
          // check if hotspot_B is none when mode set to 'A', and default to hotspot_A
          if (temp == "none") {
            hotspot_dir = hotspot_A;
          } else {
            hotspot_dir = hotspot_mode == "B" ? hotspot_B : hotspot_C;
          }
        }
        ROS_INFO("Hotspot src dir: %s", hotspot_dir.c_str());
        return true;
      }
    }
  } catch (...) {
    ROS_WARN("WARNING: %s Not Found or is Empty. Aborting hotspot check...", HOTSPOT_PATH.c_str());
  }
  return false;
}

void WaypointSelector::change_case(std::string& strtxt)
{
  // change to lower case
  for_each(strtxt.begin(), strtxt.end(), [](char& c) { c = ::tolower(c); });
}

void WaypointSelector::read_csv(std::string&& file_types, std::string& dir_location)
{
  if (file_types == "route_dict") {
    if (!route_dictionary.empty()) {
      // clear before refreshing data to avoid data corruption
      route_dictionary.clear();
    }
    try {
      io::CSVReader<2> in(dir_location);
      in.read_header(io::ignore_extra_column, "route_id", "file_dir");
      std::string route_id, file_dir;
      while (in.read_row(route_id, file_dir)) {
        change_case(route_id);
        route_dictionary[route_id] = file_dir;
      }
      ROS_INFO("Read waypoint_dir.csv");
    } catch (...) {
      ROS_ERROR("%s not Found of Failed to load correctly. Terminating waypoint_selector node...", dir_location.c_str());
    }

  } else if (file_types == "waypoints") {
    communication_msgs::RouteResponse response_msg;
    try {
      io::CSVReader<1> in(dir_location);
      in.read_header(io::ignore_extra_column, "waypoint_id");
      int waypoint_id;
      while (in.read_row(waypoint_id)) {
        last_waypoint = waypoint_id;
      }
      last_waypoint = last_waypoint - GOAL_WAYPOINT_OFFSET;
      ROS_INFO("Read waypoint.csv completed");
      // send route accepted status to job_handler

      response_msg.current_state = ROUTE_ACCEPTED;
      ROS_INFO("GOAL complete trigger set to waypoind_id: %d", last_waypoint);
    } catch (...) {
      ROS_ERROR("File not found : %s. Aborting file reading", dir_location.c_str());
      isFileMissing = true;
    }
    subtask_update_pub.publish(response_msg);
  }
}

void WaypointSelector::waypoint_changer(std::string& route)
{
  communication_msgs::RouteResponse response_msg;
  response_msg.status_message = route + " completed";
  response_msg.current_state = route;

  subtask_update_pub.publish(response_msg);
  ROS_INFO("Waypoint route file change request sent to job handler");
}

void WaypointSelector::closestWaypointCallback(const std_msgs::Int32& msg)
{
  if (isCheckClosestWaypoint) {
    int approximate_waypoint = msg.data;
    // ROS_INFO("closest waypoint: %d, last waypoint: %d", approximate_waypoint, last_waypoint);
    // debug only
    std_msgs::Int32 c_msg;
    c_msg.data = msg.data;
    closest_waypoint_pub.publish(c_msg);

    std_msgs::Int32 l_msg;
    l_msg.data = last_waypoint;
    last_waypoint_pub.publish(l_msg);
    // end of debug code

    if (approximate_waypoint >= last_waypoint) {
      ROS_INFO("End of waypoint.csv file reached...");
      last_waypoint = MAX_WAYPOINT_ID;
      if (!isRouteCompleted) {
        // This is where will implement call delay to change waypoint file
        ros::Duration(3.0).sleep();
        waypoint_changer(subtask_route_id);
      }
    }
  }
}

void WaypointSelector::route_append(std::string& route_name)
{
  ROS_INFO("route name : %s", route_name.c_str());
  std::string search_string = "route_default";
  std::string replace_string = route_name.c_str();
  std::string inbuf;
  std::fstream input_file(IN_SHELL_SCRIRT_PATH, std::ios::in);
  std::ofstream output_file(OUT_SHELL_SCRIRT_PATH);
  while (!input_file.eof()) {
    std::getline(input_file, inbuf);
    int spot = inbuf.find(search_string);
    if (spot >= 0) {
      std::string tmpstring = inbuf.substr(0, spot);
      tmpstring += replace_string;
      tmpstring += inbuf.substr(spot + search_string.length(), inbuf.length());
      inbuf = tmpstring;
    }

    output_file << inbuf << std::endl;
  }
  ROS_INFO("writing file");
}

void WaypointSelector::isBluelineCallback(const std_msgs::String& msg)
{
  if (isHotspotSet) {
    if (std::find(IGNORE_LIST.begin(), IGNORE_LIST.end(), msg.data)
        == IGNORE_LIST.end()) {
      std_msgs::Int16 error;
      if (std::find(BLUE_LINE_NG.begin(), BLUE_LINE_NG.end(), msg.data)
          != BLUE_LINE_NG.end()) {
        ROS_INFO("Hotspot Error: BLue line not detected");
        error.data = ON;
      } else {
        ROS_INFO("BLue line detected");
        error.data = OFF;
      }
      hotspot_error_pub.publish(error);
      isHotspotSet = false;
    }
  }
}

void WaypointSelector::subtaskCallback(const communication_msgs::RouteInfo& message)
{
  // int bool message.is_completed and string message.route
  // initialize all counters and state
  isRouteCompleted = message.is_completed;
  // start
  communication_msgs::RouteResponse response_msg;
  response_msg.current_state = TASK_COMPLETE;
  response_msg.status_message = "waiting";
  ROS_INFO("Received Route: %s (from job handler node)", message.route.c_str());

  // bypass closestWaypointCallback code before route load
  // This is a way to prevent reading closest waypoint data from previously completed route
  // using it to evaluate current route's completion
  isCheckClosestWaypoint = false;

  if (!isRouteCompleted) {
    // prevent route_dictionary refresh, which might cause error when attempting to access
    // and modify it at same time
    isCsvRefresh = false;

    if (route_dictionary.find(message.route) != route_dictionary.end()) {
      ROS_INFO("Starting to load waypoint data into autoware...");
      std::string file_path = route_dictionary[message.route];
      try {
        // load route
        read_csv("waypoints", file_path);
        ROS_INFO("Loading new route waypoint data");
        subtask_route_id = message.route;
        route_append(subtask_route_id);

        system(("chmod +x " + OUT_SHELL_SCRIRT_PATH).c_str());
        system(("sh " + RESPAWN_SHELL_SCRIRT_PATH).c_str());
        ROS_INFO("Done loading waypoint data");

        // check if hotspot
        if (check_hotspot(subtask_route_id)) {
          ros::Duration(1.0).sleep();
          // set hotspot
          ROS_INFO("Setting hotspot...");
          system(("bash " + SPAWN_HOTSPOT_SCRIPT + " " + hotspot_dir).c_str());
          // check if blueline is set for proper self driving operation
          if (!isSkipBluelineCheck) {
            isHotspotSet = true;
          } else {
            // reset for next trigger
            isSkipBluelineCheck = false;
          }
        }
        // re-activate closestWaypointCallback code
        ros::Duration(2.0).sleep();
        isCheckClosestWaypoint = true;
      } catch (...) {
        if (!isFileMissing) {
          ROS_ERROR("%s.csv file not found in %s. Aborting file reading", message.route.c_str(), CSV_ROUTE_PATH.c_str());
        }
        isFileMissing = false;
        communication_msgs::RouteResponse response_msg;

        // send route loading failure status to job_handler
        response_msg.current_state = ROUTE_REJECTED;
        response_msg.status_message = "waiting";
        subtask_update_pub.publish(response_msg);
      }
      isCsvRefresh = true;
    } else {
      ROS_ERROR("Route: %s not found in %s. Aborting task assignment", message.route.c_str(), CSV_ROUTE_PATH.c_str());
      subtask_update_pub.publish(response_msg);
      isCsvRefresh = true;
    }
  } else {
    // signal Freedom that task is done and ready for next task
    ROS_INFO("Route completed.... Waiting for next assignment.");
    subtask_update_pub.publish(response_msg);
  }
}

void WaypointSelector::csvRefreshTriggerCallback(const std_msgs::Int8& msg)
{
  if (msg.data == ON) {
    if (isCsvRefresh) {
      // reload route_path info in interval set by csv_refresh_clock node
      std::string route_path = CSV_ROUTE_PATH;
      read_csv("route_dict", route_path);
    }
  }
}

void WaypointSelector::run()
{

  // send initial ready message to Freedom
  communication_msgs::RouteResponse init_msg;
  init_msg.current_state = WAITING_FOR_TASK;
  init_msg.status_message = "waiting";
  ROS_INFO("Publishing initial signal to job handler node");

  // create a wait state to ensure first published message is received
  while (subtask_update_pub.getNumSubscribers() < 1) { }

  subtask_update_pub.publish(init_msg);

  // load route_id to directory csv file
  std::string route_path = CSV_ROUTE_PATH;
  read_csv("route_dict", route_path);
  //  use multithreaded spin instead, in order to process callbacks simultaneously
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_selector");

  WaypointSelector subtask_waypoint_loader;
  subtask_waypoint_loader.run();

  return 0;
}
