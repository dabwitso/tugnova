#ifndef VEHICLE_WAYPOINT_DISTANCE_CHECKER_H
#define VEHICLE_WAYPOINT_DISTANCE_CHECKER_H

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/RemoteCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <vehicle_waypoint_distance_checker/PoseStruct.h>

struct Pose
{
    double x;
    double y;
    int id;
};

class vehicleDistanceCheck
{
public:
    vehicleDistanceCheck();
    ~vehicleDistanceCheck();
    void run();
private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    // Publisher
    ros::Publisher remote_cmd_pub_;
    ros::Publisher vehicle_distance_pub_;
    //Subscriber
    ros::Subscriber safety_waypoint_sub_;
    ros::Subscriber current_pose_sub_;
    // msg
    vehicle_waypoint_distance_checker::PoseStruct vehicle_msg_; 

    Pose safety_current_pose_, safety_previous_pose_;

    double current_pose_x_;
    double current_pose_y_;
    double vehicle_waypoint_distance_;

    // functions
    void vehicleDistanceState();
    double verticalDistance(const double& current_pose_x, const double& current_pose_y);
    // callback 
    void safetyWaypointCallback(const autoware_msgs::LaneConstPtr& msg);
    void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

#endif // VEHICLE_WAYPOINT_DISTANCE_CHECKER_H
