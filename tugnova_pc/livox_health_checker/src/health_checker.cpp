#include "livox_health_checker.h"

HealthChecker::HealthChecker()
{
  // ros handles
  livox_health_pub = nh.advertise<std_msgs::Bool>("/livox_health", 1);
  livox_error0_pub = nh.advertise<std_msgs::Int32>("livox_error_0", 1);
  livox_error1_pub = nh.advertise<std_msgs::Int32>("livox_error_1", 1);
  livox_error2_pub = nh.advertise<std_msgs::Int32>("livox_error_2", 1);

  livox_ping_sub = nh.subscribe("livox_ping_status", 1, &HealthChecker::pingCallback, this);
  script_PID_sub = nh.subscribe("ping_script_pid", 1, &HealthChecker::pidCallback, this);

  // variable initialization;

  isNodesAlive = true;
  isPingError = false;
  script_pid = INT_MAX;
  counter = INIT;
  error_display_count = INIT;

  script_mode = "ping";

  NODES_TO_TRACK = { "/area_confirm_publisher", "/ray_ground_filter_2", "/points_concat_filter" };
}

void HealthChecker::pidCallback(const std_msgs::Int32& msg)
{
  script_pid = msg.data;
  ROS_INFO("PID: %d", script_pid);
}

void HealthChecker::pingCallback(const std_msgs::Int16& msg)
{
  // check if ping connection is successful
  if (msg.data == ON)
    isPingError = true;
  else
    isPingError = false;

  // check if ros node is alive
  nodeAliveChecker();

  // determine overall health based on ping and node heartbeat
  livoxHealthChecker();
}

void HealthChecker::nodeAliveChecker()
{
  if (!node_list.empty()) {
    // ROS_INFO("Clearing vector");
    node_list.clear();
  }
  if (ros::master::getNodes(node_list)) {
    for (const auto node : NODES_TO_TRACK) {
      if (counter > NUMBER_OF_NODES) {
        // reset
        isNodesAlive = true;
        counter = INIT;
      }
      if (std::find(node_list.begin(), node_list.end(), node) != node_list.end()) {
        isNodesAlive = !isNodesAlive ? false : true;
      } else
        isNodesAlive = false;
      ++counter;
    }
  } else {
    ROS_ERROR("Node list empty. Failed to get list of nodes registered with ros_master");
    isNodesAlive = false;
  }
}

void HealthChecker::livoxHealthChecker()
{
  std_msgs::Bool state_msg;
  std_msgs::Int32 error_msg;

  if (isNodesAlive) {
    error_display_count = INIT;
    if (isPingError) {
      state_msg.data = false;
      error_msg.data = PING_ERROR;
      livox_error0_pub.publish(error_msg);
      livox_error1_pub.publish(error_msg);
      livox_error2_pub.publish(error_msg);
    } else
      state_msg.data = true;
  } else {
    state_msg.data = false;
    error_msg.data = NODE_ERROR;
    livox_error0_pub.publish(error_msg);
    livox_error1_pub.publish(error_msg);
    livox_error2_pub.publish(error_msg);
    // display jetson dead notice on tugnova's screen
    if (error_display_count == INIT){
      script_mode = "display";
      system(("bash " + LIVOX_SCRIPT_PATH + " " + script_mode).c_str());
      ++error_display_count;
    }
  }

  livox_health_pub.publish(state_msg);
}

void HealthChecker::run()
{
  script_mode = "ping";
  system(("bash " + LIVOX_SCRIPT_PATH + " " + script_mode + " " + LIVOX_NODE_IP).c_str());
  ros::spin();
  system(("kill -9 " + std::to_string(script_pid)).c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "livox_health_checker");
  HealthChecker health_check;
  health_check.run();

  // terminate ping script using pid
  return 0;
}
