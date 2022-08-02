#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "autoware_msgs/LaneArray.h"
#include "autoware_config_msgs/ConfigWaypointFollower.h"

autoware_msgs::Waypoint now_wp;
autoware_msgs::Waypoint wp_;
autoware_config_msgs::ConfigWaypointFollower WaypointFollower_;

ros::Publisher pub_config_waypoint_follower;

bool initflag = false;

void settingWaypointFollower_(const autoware_config_msgs::ConfigWaypointFollower &WaypointFollower)
{
    WaypointFollower_ = WaypointFollower;
}

void safetyWaypointCallback(const autoware_msgs::Lane &lane)
{
    //ルートファイルから取得したtarget_point_ratioを設定する
    WaypointFollower_.lookahead_ratio = lane.waypoints[0].wpc.target_point_ratio;
    //ルートファイルから取得したminimum_target_point_distanceを設定する
    WaypointFollower_.minimum_lookahead_distance = lane.waypoints[0].wpc.min_tpd;
    pub_config_waypoint_follower.publish(WaypointFollower_);
    ros::spinOnce();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "change_config_waypoint_follower");
    ros::NodeHandle n;
    ros::Subscriber sub_ = n.subscribe("config/waypoint_follower", 10, settingWaypointFollower_);
    ros::Subscriber safetywaypoints_sub = n.subscribe("safety_waypoints", 10, safetyWaypointCallback);

    ros::NodeHandle nh;
    pub_config_waypoint_follower = nh.advertise<autoware_config_msgs::ConfigWaypointFollower>("config/waypoint_follower", 10);

    ros::spin();
    return 0;
}
