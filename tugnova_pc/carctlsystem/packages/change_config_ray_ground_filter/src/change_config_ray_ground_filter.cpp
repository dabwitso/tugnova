#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "autoware_msgs/LaneArray.h"
#include "autoware_config_msgs/ConfigRayGroundFilter.h"

autoware_msgs::Waypoint now_wp;
autoware_msgs::Waypoint wp_;
autoware_config_msgs::ConfigRayGroundFilter RayGroundFilter_;

ros::Publisher pub_config_ray_ground_filter;

bool initflag = false;

void settingRayGroundFilter_(const autoware_config_msgs::ConfigRayGroundFilter &RayGroundFilter)
{
  RayGroundFilter_ = RayGroundFilter;
}

void safetyWaypointCallback(const autoware_msgs::Lane &lane)
{
  //ルートファイルから取得したlocal_max_slopeを設定する
  RayGroundFilter_.local_max_slope = lane.waypoints[0].wpc.local_max_slope;
  pub_config_ray_ground_filter.publish(RayGroundFilter_);
  ros::spinOnce();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_config_ray_ground_filter");
  ros::NodeHandle n;
  ros::Subscriber sub_ = n.subscribe("config/ray_ground_filter", 10, settingRayGroundFilter_);
  ros::Subscriber safetywaypoints_sub = n.subscribe("safety_waypoints", 10, safetyWaypointCallback);

  ros::NodeHandle nh;
  pub_config_ray_ground_filter = nh.advertise<autoware_config_msgs::ConfigRayGroundFilter>("config/ray_ground_filter", 10);

  ros::spin();
  return 0;
}
