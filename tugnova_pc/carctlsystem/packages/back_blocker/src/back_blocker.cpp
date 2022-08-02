#include <ros/ros.h>
#include <sstream>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "autoware_msgs/LaneArray.h"
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <carctl_msgs/monitor_status.h>

autoware_msgs::Lane now_safety_waypoint;
bool now_safety_waypoint_initialized = false;
int waypointid_lag_threshold = 1;
ros::Publisher pub_waypoints;

const int16_t ERROR = 1;
int16_t monitor_status = 0;

void WaypointsCallback(const autoware_msgs::Lane &original_safety_waypoints)
{
  //保持しているsafety_waypointが
  if (now_safety_waypoint_initialized == false)
  {
    //存在しない場合
    now_safety_waypoint_initialized = true;
    //safety_waypointを受信したoriginal_safety_waypointで更新する
    now_safety_waypoint = original_safety_waypoints;
  }
  else
  {
    //存在する場合
    int original_waypoint_id = original_safety_waypoints.waypoints[0].wpc.waypoint_id;
    int now_waypoint_id = now_safety_waypoint.waypoints[0].wpc.waypoint_id;
    //受信したwaypointが現在のwaypointよりも
    if (original_waypoint_id >= now_waypoint_id)
    {
      //前方のwaypointである場合
      //保持しているsafety_waypointを受信したoriginal_safety_waypointで更新する
      now_safety_waypoint = original_safety_waypoints;
    }
    else
    {
      //後方のwaypointである場合
      //保持しているwaypointから設定値以上waypointIDが
      if (now_waypoint_id - original_waypoint_id > waypointid_lag_threshold)
      {
        //ずれた場合
        //保持しているsafety_waypointを受信したoriginal_safety_waypointで更新する
        now_safety_waypoint = original_safety_waypoints;
      }
      else
      {
        //ずれない場合
        //保持しているsafety_waypointのヘッダー情報を受信したoriginal_safety_waypointのヘッダー情報で更新する
        now_safety_waypoint.header = original_safety_waypoints.header; 
      }
    }
  }

  //保持しているsafety_waypointを出力する
  if (monitor_status != ERROR) {
    pub_waypoints.publish(now_safety_waypoint);
  }
}

/** /monitor_statusの受信. */
void monitorStatusCallback(const carctl_msgs::monitor_status::ConstPtr& msg) {
  if (msg->service_name != "lost_localization") { return; }
  monitor_status = msg->status;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "back_blocker");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("waypointid_lag_threshold", waypointid_lag_threshold);
  pub_waypoints = n.advertise<autoware_msgs::Lane>("safety_waypoints", 10);

  ros::Subscriber sub_waypoints = n.subscribe("original_safety_waypoints", 10, WaypointsCallback);
  ros::Subscriber sub_monitor_status = n.subscribe("/monitor_status", 10, monitorStatusCallback);
  ros::spin();
  return 0;
}
