#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "autoware_msgs/LaneArray.h"
#include "autoware_config_msgs/ConfigVoxelGridFilter.h"

autoware_msgs::Waypoint now_wp;
autoware_msgs::Waypoint wp_;
autoware_config_msgs::ConfigVoxelGridFilter VoxelGridFilter_;

ros::Publisher pub_config_voxel_grid_filter;

bool initflag = false;

void settingVoxelGridFilter_(const autoware_config_msgs::ConfigVoxelGridFilter &VoxelGridFilter)
{
  VoxelGridFilter_ = VoxelGridFilter;
}

void safetyWaypointCallback(const autoware_msgs::Lane &lane)
{
  //ルートファイルから取得したvoxel_leaf_sizeを設定する
  VoxelGridFilter_.voxel_leaf_size = lane.waypoints[0].wpc.voxel_leaf_size;
  pub_config_voxel_grid_filter.publish(VoxelGridFilter_);
  ros::spinOnce();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_config_voxel_grid_filter");
  ros::NodeHandle n;
  ros::Subscriber sub_ = n.subscribe("config/voxel_grid_filter", 10, settingVoxelGridFilter_);
  ros::Subscriber safetywaypoints_sub = n.subscribe("safety_waypoints", 10, safetyWaypointCallback);

  ros::NodeHandle nh;
  pub_config_voxel_grid_filter = nh.advertise<autoware_config_msgs::ConfigVoxelGridFilter>("config/voxel_grid_filter", 10);

  ros::spin();
  return 0;
}
