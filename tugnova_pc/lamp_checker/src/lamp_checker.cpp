#include "lamp_checker.h"

LampState::LampState()
{
  // ros handles
  lamp_state_pub = nh.advertise<communication_msgs::LampState>("/lamp_state", 1);
  drive_stop_pub = nh.advertise<std_msgs::Int16>("/stop_drive", 1);
  drive_start_pub = nh.advertise<std_msgs::Int16>("/start_drive", 1);

  hmi_lamp_sub = nh.subscribe("/hmi_lamp_stop_cmd", 10, &LampState::hmiLampCallback, this);
  lamp_flag_sub = nh.subscribe("/safety_waypoints", 10, &LampState::lampWaypointCallback, this);

  isSpecialStop = false;
  publish_lock = OFF;
}

void LampState::hmiLampCallback(const std_msgs::Int16& msg)
{
  if (msg.data == ON) {
    ROS_INFO("Stop = true");
    isSpecialStop = true;
  } else if (msg.data == OFF) {
    ROS_INFO("Stop = false");
    isSpecialStop = false;
  }
}

void LampState::lampWaypointCallback(const autoware_msgs::Lane& msg)
{
  int id = msg.waypoints[NOW].wpc.lamp_id;

  if (id) {
    if (id != SPECIAL_STOP_FLAG) {
      communication_msgs::LampState status_msg;
      status_msg.lamp_id = id % 2 ? ((id - 1) / ID_NORMALIZER_FACTOR)
                                  : (id / ID_NORMALIZER_FACTOR);
      if (status_msg.lamp_id != SPECIAL_LAMP_ID) {
        status_msg.status = id % 2 ? ON : OFF;
        //ROS_INFO("Lamp ID: %d,  status: %d", status_msg.lamp_id, status_msg.status);
      }
      // only publish signal to HMI once
      if (publish_lock == OFF) {
        lamp_state_pub.publish(status_msg);
        publish_lock = ON;
      }
    } else if (id == SPECIAL_STOP_FLAG) {
      std_msgs::Int16 drive_cmd;
      drive_cmd.data = ON;

      if (publish_lock == OFF && isSpecialStop) {
        ROS_INFO("Tugnova stopped by freedom at special lamp id");
        drive_stop_pub.publish(drive_cmd);
        publish_lock = ON;
      } else if (publish_lock == ON && !isSpecialStop) {
        ROS_INFO("Tugnova started by freedom at special lamp id");
        drive_start_pub.publish(drive_cmd);
        publish_lock = OFF;
      }
    }
  } else
    publish_lock = OFF;
}

void LampState::run()
{
  ROS_INFO("Lamp checker activate...");
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lamp_checker");
  LampState lamp;
  lamp.run();

  return 0;
}
