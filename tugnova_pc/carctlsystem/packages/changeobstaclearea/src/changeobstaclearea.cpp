#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "autoware_msgs/LaneArray.h"
#include "autoware_config_msgs/ConfigVelocitySet.h"

autoware_config_msgs::ConfigVelocitySet VelocitySet_;

ros::Publisher pub_config_velocityset;


void settingVelocitySet_(const autoware_config_msgs::ConfigVelocitySet &VelocitySet)
{
    VelocitySet_ = VelocitySet;
}

void safetyWaypointCallback(const autoware_msgs::Lane &lane)
{
    //ルートファイルから取得した物体検知時前方停止距離を設定する
    VelocitySet_.stop_distance_obstacle = lane.waypoints[0].wpc.sdo;
    pub_config_velocityset.publish(VelocitySet_);
    ros::spinOnce();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "changeobstaclearea");
    ros::NodeHandle n;
    ros::Subscriber sub_ = n.subscribe("config/velocity_set", 10, settingVelocitySet_);
    ros::Subscriber safetywaypoints_sub = n.subscribe("safety_waypoints", 10, safetyWaypointCallback);

    ros::NodeHandle nh;
    pub_config_velocityset = nh.advertise<autoware_config_msgs::ConfigVelocitySet>("config/velocity_set", 10);

    ros::spin();
    return 0;
}
