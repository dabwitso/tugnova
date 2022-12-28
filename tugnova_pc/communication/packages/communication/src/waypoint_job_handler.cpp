#include "waypoint_job_handler.h"

JobScheduler::JobScheduler()
    : private_nh("~")
{
  // ros handles
  // publication
  route_cmd_pub = nh.advertise<communication_msgs::RouteInfo>("route_cmds", 10);
  reply_server_pub = nh.advertise<std_msgs::String>("tagnova_response", 10);
  keep_tugnova_stopped_pub = nh.advertise<std_msgs::Int16>("stop_drive", 10);

  // subscription
  subtask_feedback_sub = nh.subscribe("route_request_fbk", 10, &JobScheduler::subtaskUpdateCallback, this);
  server_request_sub = nh.subscribe("server_request", 10, &JobScheduler::routeCmdCallback, this);
  battery_level_sub = nh.subscribe("battery_range", 10, &JobScheduler::batteryLevelCallback, this);
  csv_refresh_clock_sub = nh.subscribe("csv_refresh", 10, &JobScheduler::csvRefreshTriggerCallback, this);

  // variables
  number_of_subtasks = INIT;
  server_cmd = DEFAULT;
  battery_level = DEFAULT;
  param_val = DEFAULT;
  job_status_message = DEFAULT_MSG;
  isCsvRefresh = true;

  tugnova_states = { "Available", "Completed", "Accepted", "Rejected" };

  // set initial default parameter
  private_nh.setParam("current_route", param_val);
}

void JobScheduler::split_string(const std::string& s, char delimiter, std::string dict_key)
{
  std::string split;
  std::istringstream ss(s);
  change_case(dict_key); // change to lower case
  int i = 0;
  while (std::getline(ss, split, delimiter)) {
    change_case(split);
    route_info[dict_key][i] = split;
    i++;
  }
}

void JobScheduler::csvRefreshTriggerCallback(const std_msgs::Int8& msg)
{
  if (msg.data == ON) {
    // load route map info in intervals determined by csv_refresh_clock node
    if (isCsvRefresh) {
      read_csv();
    }
  }
}

void JobScheduler::read_csv()
{
  ROS_INFO("Read route_map.csv");
  if (!route_info.empty()) {
    // clear route_info map before loading new data to ensure no data corruption
    route_info.clear();
  }
  try {
    io::CSVReader<2> in(ROUTE_MAP_PATH);
    in.read_header(io::ignore_extra_column, "destination", "route_link");
    std::string destination, route_link;
    while (in.read_row(destination, route_link)) {
      split_string(route_link, '/', destination);
    }
  } catch (...) {
    ROS_ERROR("%s not Found or failed to load. Terminating Waypoint_job_handler node...", ROUTE_MAP_PATH.c_str());
  }
}

void JobScheduler::route_update(std::string previous_state)
{

  // get route list and assign next route
  for (int i = 0; i < number_of_subtasks; ++i) {
    if (route_info[server_cmd][i] == previous_state) {
      communication_msgs::RouteInfo msg_cmd;
      std::string next_state = route_info[server_cmd][i + 1]; // load next route
      ROS_INFO("Calculating next route from waypoint_map.csv");
      if (next_state == "finish") {
        ROS_INFO("Sending finish cmd to waypoint_selector");
        msg_cmd.is_completed = true;
        msg_cmd.route = next_state;
        route_cmd_pub.publish(msg_cmd);

        ROS_INFO("Server published %s command to Tagnova ", next_state.c_str());

        // update route info in parameter server
        param_val = DEFAULT;
        private_nh.setParam("current_route", param_val);
      } else {
        // prevent tugnova from moving after receiving route as requested by Freedom
        std_msgs::Int16 drive_stop;
        drive_stop.data = 1;
        keep_tugnova_stopped_pub.publish(drive_stop);
        ros::Duration(2.0).sleep();

        msg_cmd.is_completed = false;
        msg_cmd.route = next_state; // load next route
        route_cmd_pub.publish(msg_cmd);
        ROS_INFO("Server published %s command to Tagnova ", next_state.c_str());

        // update route info in parameter server
        param_val = next_state;
        private_nh.setParam("current_route", param_val);
      }
    }
  }
}

void JobScheduler::change_case(std::string& strtxt)
{
  for_each(strtxt.begin(), strtxt.end(), [](char& c) { c = ::tolower(c); });
}

void JobScheduler::assign_job()
{
  // sends which route waypoint data to load by waypoint_selector node
  communication_msgs::RouteInfo msg_cmd;
  msg_cmd.is_completed = false;
  // check if route_to_** command is in map
  if (route_info.find(server_cmd) != route_info.end()) {
    msg_cmd.route = route_info[server_cmd][0]; // assign initial route command

    // reject job if starting route is requested at low battery
    if (battery_level == "low" && msg_cmd.route == STARTING_ROUTE) {
      std_msgs::String msg;
      msg.data = "LOW_BATTERY";
      reply_server_pub.publish(msg);
      ROS_ERROR("Job [%s] aborted. Low battery warning", server_cmd.c_str());
    } else {
      number_of_subtasks = sizeof(route_info[server_cmd]) / sizeof(route_info[server_cmd][0]);
      // save route info to parameter server
      param_val = route_info[server_cmd][0];
      private_nh.setParam("current_route", param_val);

      // prevent tugnova from moving after receiving route as requested by Freedom
      std_msgs::Int16 drive_stop;
      drive_stop.data = 1;
      keep_tugnova_stopped_pub.publish(drive_stop);

      ros::Duration(2.0).sleep();
      route_cmd_pub.publish(msg_cmd);
      std_msgs::String msg;
      msg.data = "Accepted";
      reply_server_pub.publish(msg);
    }
  } else {
    ROS_ERROR("Route command %s not found in %s", server_cmd.c_str(), ROUTE_MAP_PATH.c_str());
    std_msgs::String msg;
    msg.data = "Invalid_job_name";
    reply_server_pub.publish(msg);
  }
  // unlock ability to refresh csv files read
  isCsvRefresh = true;
}

void JobScheduler::routeCmdCallback(const std_msgs::String& msg)
{
  // responsible for receiving server commands
  // prevent csv file refresh to avoid conflict when trying to access file while parsing
  isCsvRefresh = false;
  server_cmd = msg.data;
  change_case(server_cmd); // change server_cmd to lower case
  assign_job();            // start job assignment using route from server
}

void JobScheduler::subtaskUpdateCallback(const communication_msgs::RouteResponse& msg)
{
  std_msgs::String state_msg;
  if (std::find(tugnova_states.begin(), tugnova_states.end(), msg.current_state) != tugnova_states.end()) {
    state_msg.data = msg.current_state;
    if (state_msg.data == tugnova_states[1])
      server_cmd = ""; //reset when task is completed
  } else {
    // vehicle is still executing task
    state_msg.data = "Busy";
    route_update(msg.current_state);
  }
  // publish to HMI server multiple times to ensure state is received
  for (int send_counter = 0; send_counter < STATUS_SEND_COUNT; ++send_counter) {
    reply_server_pub.publish(state_msg);
    // apply 1 sec interval between transmissions
    ros::Duration(1.0).sleep();
  }
}

void JobScheduler::batteryLevelCallback(const std_msgs::String& msg)
{
  battery_level = msg.data;
}

void JobScheduler::run()
{
  read_csv(); // load route map info
  ros::spin();
  // delete parameter before shutting down node
  private_nh.deleteParam("current_route");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_job_handler");

  JobScheduler job_handler;
  job_handler.run();
  return 0;
}
