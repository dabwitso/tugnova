#include "waypoint_job_handler.h"

JobScheduler::JobScheduler()
    : private_nh("~")
{
  // ros handles
  // publication
  route_cmd_pub = nh.advertise<communication_msgs::RouteInfo>("route_cmds", 10);
  reply_server_pub = nh.advertise<std_msgs::String>("tagnova_response", 10);

  // subscription
  subtask_feedback_sub = nh.subscribe("route_request_fbk", 10, &JobScheduler::subtaskUpdateCallback, this);
  server_request_sub = nh.subscribe("server_request", 10, &JobScheduler::routeCmdCallback, this);
  battery_level_sub = nh.subscribe("battery_range", 10, &JobScheduler::batteryLevelCallback, this);

  // variables
  number_of_subtasks = INIT;
  server_cmd = DEFAULT;
  battery_level = DEFAULT;
  job_status_message = DEFAULT_MSG;
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

void JobScheduler::read_csv()
{

  io::CSVReader<2> in(ROUTE_MAP_PATH);
  in.read_header(io::ignore_extra_column, "destination", "route_link");
  std::string destination, route_link;
  while (in.read_row(destination, route_link)) {
    split_string(route_link, '/', destination);
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
      } else {

        msg_cmd.is_completed = false;
        msg_cmd.route = next_state; // load next route
        route_cmd_pub.publish(msg_cmd);
        ROS_INFO("Server published %s command to Tagnova ", next_state.c_str());
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
    number_of_subtasks = sizeof(route_info[server_cmd]) / sizeof(route_info[server_cmd][0]);
    route_cmd_pub.publish(msg_cmd);
    std_msgs::String msg;
    msg.data = "Accepted";
    reply_server_pub.publish(msg);
  } else {
    ROS_ERROR("Route command %s not found in %s", server_cmd.c_str(), ROUTE_MAP_PATH);
    std_msgs::String msg;
    msg.data = "Invalid_job_name";
    reply_server_pub.publish(msg);
  }
}

void JobScheduler::routeCmdCallback(const std_msgs::String& msg)
{
  // responsible for receiving server commands
  if (battery_level != "low") {
    server_cmd = msg.data;
    change_case(server_cmd); // change server_cmd to lower case
    assign_job();  // start job assignment using route from server
  } else {
    std_msgs::String msg;
    msg.data = "LOW_BATTERY";
    reply_server_pub.publish(msg);
    ROS_ERROR("Job [%s] aborted. Low battery warning", server_cmd.c_str());
  }
}

void JobScheduler::subtaskUpdateCallback(const communication_msgs::RouteResponse& message)
{
  // int message.tagnovaID and string message.response
  if (message.current_state == "ready") {
    job_status_message = message.status_message;
    ROS_INFO("Tagnova status: %s ...", job_status_message.c_str());
    std_msgs::String msg;
    msg.data = job_status_message;
    reply_server_pub.publish(msg);
  } else if (message.current_state == "done") {
    job_status_message = message.status_message;
    std_msgs::String msg;
    msg.data = "Completed";
    reply_server_pub.publish(msg);
    // for information only. comment out if not needed
    job_status_message = message.status_message;
    ROS_INFO("Tagnova status: %s ...", job_status_message.c_str());
    server_cmd = ""; // reset
  } else {
    job_status_message = message.status_message;
    ROS_INFO("Tagnova status: %s ...", job_status_message.c_str());
    std_msgs::String msg;
    msg.data = "busy";
    reply_server_pub.publish(msg);

    route_update(message.current_state);
  }
}

void JobScheduler::batteryLevelCallback(const std_msgs::String& msg)
{
  battery_level = msg.data;
}

void JobScheduler::run()
{
  ros::Rate loop_rate(READ_CSV_REFRESH_RATE);
  while (ros::ok()) {
    read_csv(); // load route map info
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_job_handler");

  JobScheduler job_handler;
  job_handler.run();
  return 0;
}
