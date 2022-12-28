#include "livox_detector.h"

LivoxDetector::LivoxDetector()
{
  // ros handles
  // publication
  second_stop_pub = nh.advertise<std_msgs::Int16>("/second_stop_flg", 1);
  livox_state_pub = nh.advertise<std_msgs::Int16>("/livox_state", 10);

  // subscription
  second_stop_waypoint_sub = nh.subscribe("/safety_waypoints", 1, &LivoxDetector::secondStopWaypointCallback, this);
  livox_result_sub = nh.subscribe("/clearing_area_bool", 10, &LivoxDetector::livoxResultCallback, this);
  second_stop_flg_sub = nh.subscribe("/livox_two_stop", 1, &LivoxDetector::secondStopActivator, this);

  // variable initialization
  isSecondStop = false;
}

void LivoxDetector::secondStopActivator(const std_msgs::Bool& msg)
{
  isSecondStop = msg.data;
}

void LivoxDetector::livoxResultCallback(const std_msgs::Bool& msg)
{
  // ROS_INFO("Livox flag: " << std::to_string(msg.data));
  if (msg.data)
    livox_stop_state.data = OFF;
  else
    livox_stop_state.data = ON;
  livox_state_pub.publish(livox_stop_state);
}

void LivoxDetector::secondStopWaypointCallback(const autoware_msgs::Lane& msg)
{
  int OD_flag = msg.waypoints[NOW].wpc.object_detection_intersect;

  if (isSecondStop && OD_flag == SECOND_STOP_FLAG) {
    ROS_INFO("Stopped at secondary stop location");
    // send stop command to drive_stop_controller node
    std_msgs::Int16 second_stop_cmd;
    second_stop_cmd.data = ON;
    second_stop_pub.publish(second_stop_cmd);

    // send livox detection to object_detection_checker node
    livox_stop_state.data = ON;
    livox_state_pub.publish(livox_stop_state);
    // reset isSecondStop
    isSecondStop = false;
  }
}

void LivoxDetector::run()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "livox_detector");
  LivoxDetector detector;
  detector.run();

  return 0;
}
