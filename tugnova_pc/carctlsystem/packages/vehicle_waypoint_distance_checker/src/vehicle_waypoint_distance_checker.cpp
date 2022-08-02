#include <ros/ros.h>
#include <tf/tf.h>

#include <autoware_msgs/LaneArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/RemoteCmd.h>

ros::Publisher remote_cmd_pub;
double current_waypoint_x = 0.0;
double current_waypoint_y = 0.0;
double distance = 1.0;

/**
 * 近似のWaypointを得る.
 */
void safetyWaypointsCallback(const autoware_msgs::Lane& msg) {
    current_waypoint_x = msg.waypoints[0].pose.pose.position.x;
    current_waypoint_y = msg.waypoints[0].pose.pose.position.y;
}

/**
 * 自己位置とWaypointの距離を求め、得られた距離が閾値以上であった場合に脱線判定を流す.
 */
void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    tf::Vector3 v1(current_waypoint_x, current_waypoint_y, 0);
    tf::Vector3 v2(msg->pose.position.x, msg->pose.position.y, 0);

    if (distance < tf::tfDistance(v1, v2)) {
        autoware_msgs::RemoteCmd remote_cmd;
        remote_cmd.vehicle_cmd.emergency = true;
        remote_cmd.control_mode = 3; // 経路外判定であることをlost_localizationへ伝える
        remote_cmd_pub.publish(remote_cmd);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "vehicle_waypoint_distance_checker");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<double>("distance", distance, 1.0);

    ros::Subscriber safety_waypoints_sub = nh.subscribe("/safety_waypoints", 100, safetyWaypointsCallback);
    ros::Subscriber current_pose_sub = nh.subscribe("/current_pose", 100, currentPoseCallback);
    remote_cmd_pub = nh.advertise<autoware_msgs::RemoteCmd>("remote_cmd", 10);

    ros::spin();

    return 0;
}