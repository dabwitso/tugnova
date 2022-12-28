#include "shutter_controller.h"

ShutterController::ShutterController()
{
  // ros handles
  // publication
  shutter_stop_pub = nh.advertise<std_msgs::Int16>("stop_drive", 1);

  // subscription
  shutter_waypoint_flg_sub = nh.subscribe("safety_waypoints", 10, &ShutterController::shutterFlgsCallback, this);

  // variable initialization
  shutterStopCount = INIT;
}

void ShutterController::shutterStopHandler()
{
  if (shutterStopCount == INIT) {
    std_msgs::Int16 stop_cmd;
    stop_cmd.data = ON;
    shutter_stop_pub.publish(stop_cmd);
    ++shutterStopCount;
  }
}

void ShutterController::shutterFlgsCallback(const autoware_msgs::Lane& msg)
{
  if (msg.waypoints[NOW].wpc.shutter_point == ON) {
    ROS_INFO("Shutter stop point reached.");
    shutterStopHandler();
  } else {
    shutterStopCount = INIT;
  }
}

void ShutterController::run()
{
  ROS_INFO("shutter controller online...");
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shutter_controller");
  ShutterController shutter_controller;
  shutter_controller.run();

  return 0;
}
