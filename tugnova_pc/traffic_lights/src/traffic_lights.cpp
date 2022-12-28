#include "traffic_lights.h"

Traffic_Lights::Traffic_Lights()
    : private_nh("~")
{
  // ros handles
  // publishers
  traffic_light_flg_pub = nh.advertise<std_msgs::Int16>("traffic_light_flg",1);
  traffic_stop_flg_pub = nh.advertise<std_msgs::Int16>("stop_drive",10);

  // subscribers
  traffic_flg_sub = nh.subscribe("/safety_waypoints", 10, &Traffic_Lights::trafficLightCallback, this);

  // variable initialization
  traffic_flg_publish_count = INIT;
  traffic_light_flg = INIT;
  TRAFFIC_LIGHT_STOP_FLAG = {1, 3, 5};
}

void Traffic_Lights::trafficLightCallback(const autoware_msgs::Lane& msg)
{
  int trl_flag = msg.waypoints[NOW].wpc.traffic_light_flg;

  if (trl_flag == OFF)
    traffic_flg_publish_count = OFF;
  else {
    if (traffic_flg_publish_count == INIT) {
      ROS_INFO("traffic light flag: %d   traffic_flg_publish_count: %d", trl_flag, traffic_flg_publish_count);
      if (*(std::find(TRAFFIC_LIGHT_STOP_FLAG.begin(), TRAFFIC_LIGHT_STOP_FLAG.end(), trl_flag)) == trl_flag) {
        // stop drive if at traffic light stop point
        ROS_INFO("Stopping tagnova...");
        std_msgs::Int16 stop_cmd;
        stop_cmd.data = ON;
        traffic_stop_flg_pub.publish(stop_cmd);
      }
      std_msgs::Int16 traffic_msg;
      traffic_msg.data = trl_flag;
      traffic_light_flg_pub.publish(traffic_msg);
      ++traffic_flg_publish_count;
    }
  }
}

void Traffic_Lights::run()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traffic_light_detector");
  Traffic_Lights traffic_control;
  traffic_control.run();
  return 0;
}
