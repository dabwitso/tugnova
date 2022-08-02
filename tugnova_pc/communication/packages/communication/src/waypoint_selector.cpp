#include "waypoint_selector.h"

WaypointSelector::WaypointSelector()
    : private_nh("~")
{
  // ros handles
  // publication
  subtask_update_pub = nh.advertise<communication_msgs::RouteResponse>("route_request_fbk", 10);

  // subscription
  subtask_cmd_sub = nh.subscribe("route_cmds", 10, &WaypointSelector::subtaskCallback, this);
  waypoint_sub = nh.subscribe("closest_waypoint", 10, &WaypointSelector::closestWaypointCallback, this);

  // variable initialization
  last_waypoint = MAX_WAYPOINT_ID;
  subtask_route_id = DEFAULT;
  is_route_completed = false;
}

void WaypointSelector::change_case(std::string& strtxt){
  // change to lower case
  for_each(strtxt.begin(), strtxt.end(),[](char& c){ c = ::tolower(c); });
}

void WaypointSelector::read_csv(std::string& file_types, std::string& dir_location)
{

  if (file_types == "route_dict") {
    io::CSVReader<2> in(dir_location);
    in.read_header(io::ignore_extra_column, "route_id", "file_dir");
    std::string route_id, file_dir;
    while (in.read_row(route_id, file_dir)) {
      change_case(route_id);
      route_dictionary[route_id] = file_dir;
    }
    ROS_INFO("CSV data read");
  } else if (file_types == "waypoints") {
    io::CSVReader<1> in(dir_location);
    in.read_header(io::ignore_extra_column, "waypoint_id");
    int waypoint_id;
    while (in.read_row(waypoint_id)) {
      last_waypoint = waypoint_id;
    }
    last_waypoint = last_waypoint - 4;
    ROS_INFO("Adjusted last waypoint id: %d", last_waypoint);
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
  int approximate_waypoint = msg.data;
  // ROS_INFO("closest waypoint: %d, last waypoint: %d", approximate_waypoint, last_waypoint);
  if (approximate_waypoint == last_waypoint) {
    ROS_INFO("End of waypoint.csv file reached...");
    last_waypoint = MAX_WAYPOINT_ID;
    if (!is_route_completed) {
      // This is where will implement call delay to change waypoint file
      ros::Duration(3.0).sleep();
      waypoint_changer(subtask_route_id);
    }
  }
}

void WaypointSelector::route_append(std::string& route_name)
{
  ROS_INFO("route name : %s", route_name.c_str());
  std::string search_string = "route_default";
  std::string replace_string = route_name.c_str();
  std::string inbuf;
  fstream input_file(IN_SHELL_SCRIRT_PATH, ios::in);
  ofstream output_file(OUT_SHELL_SCRIRT_PATH);
  while (!input_file.eof()) {
    getline(input_file, inbuf);
    int spot = inbuf.find(search_string);
    if (spot >= 0) {
      std::string tmpstring = inbuf.substr(0, spot);
      tmpstring += replace_string;
      tmpstring += inbuf.substr(spot + search_string.length(), inbuf.length());
      inbuf = tmpstring;
    }
    // remove old file before generating new file
    // std::string cmd = "rm "+ OUT_SHELL_SCRIRT_PATH;
    // system(cmd);

    output_file << inbuf << endl;
  }
  ROS_INFO("writing file");
  //ros::Duration(3.0).sleep();
}

void WaypointSelector::subtaskCallback(const communication_msgs::RouteInfo& message)
{
  // int bool message.is_completed and string message.route
  // initialize all counters and state
  is_route_completed = message.is_completed;
  // start
  communication_msgs::RouteResponse response_msg;
  response_msg.current_state = "done";
  response_msg.status_message = "waiting";
  ROS_INFO("Received Route: %s (from job handler node)", message.route.c_str());
  if (!is_route_completed) {
    if (route_dictionary.find(message.route) != route_dictionary.end()) {
      ROS_INFO("Starting to load waypoint data into autoware...");
      std::string file_path = route_dictionary[message.route];
      try {
        read_csv("waypoints", file_path);
        ROS_INFO("Loading new route waypoint data");
        subtask_route_id = message.route;
        route_append(subtask_route_id);

        std::string cmd = "chmod +x " + OUT_SHELL_SCRIRT_PATH;
        system(cmd);
        // system("rm /home/nvidia/Autoware/ros/nodeshell/12waypointloader.sh");
        // system("mv /home/nvidia/Autoware/ros/src/communication/packages/communication/src/12waypointloader.sh /home/nvidia/Autoware/ros/nodeshell/");
        ros::Duration(2.0).sleep();
        cmd = "sh " + RESPAWN_SHELL_SCRIRT_PATH;
        system(cmd);
        ROS_INFO("Done loading waypoint data");
      } catch (...) {

        ROS_ERROR("%s.csv file not found in %s. Aborting file reading", message.route.c_str(), CSV_ROUTE_PATH);
        communication_msgs::RouteResponse response_msg;
        response_msg.current_state = "done";
        response_msg.status_message = "waiting";
        subtask_update_pub.publish(response_msg);
        //ros::Duration(5.0).sleep();
      }
    } else {
      ROS_ERROR("Route: %s not found in %s. Aborting task assignment", message.route.c_str(), CSV_ROUTE_PATH);
      subtask_update_pub.publish(response_msg);
      //ros::Duration(5.0).sleep();
    }
  } else {
    // signal Freedom that task is done and ready for next task
    ROS_INFO("Route completed.... Waiting for next assignment.");
    subtask_update_pub.publish(response_msg);
    //ros::Duration(5.0).sleep();
  }
}

WaypointSelector::run()
{

  // send initial ready message to Freedom
  communication_msgs::RouteResponse init_msg;
  init_msg.current_state = "ready";
  init_msg.status_message = "waiting";
  ROS_INFO("Publishing initial signal to job handler node");

  // create a wait state to ensure first published message is received
  while (subtask_update_pub.getNumSubscribers() < 1) { }

  subtask_update_pub.publish(init_msg);

  ros::rate loop_rate(READ_CSV_ROUTE_DIR_REFRESH_RATE);
  while (ros::ok()) {
    // load route_id to directory csv file
    read_csv("route_dict", CSV_ROUTE_PATH);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_selector");

  WaypointSelector subtask_waypoint_loader;
  subtask_waypoint_loader.run();

  return 0;
}
