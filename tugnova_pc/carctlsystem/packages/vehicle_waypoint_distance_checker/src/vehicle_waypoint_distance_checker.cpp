#include "vehicle_waypoint_distance_checker.h"

vehicleDistanceCheck::vehicleDistanceCheck()
    : private_nh("~")
    , current_pose_x_(0.0)
    , current_pose_y_(0.0)
    , vehicle_waypoint_distance_(0.0)
{
    // ros param
    private_nh.param<double>("distance", vehicle_waypoint_distance_, 1.0);

    // ros handles
    remote_cmd_pub_ = nh.advertise<autoware_msgs::RemoteCmd>("remote_cmd", 10);
    vehicle_distance_pub_ = nh.advertise<vehicle_waypoint_distance_checker::PoseStruct>("vehicle_distance", 10);
    safety_waypoint_sub_ = nh.subscribe("safety_waypoints", 10, &vehicleDistanceCheck::safetyWaypointCallback, this);
    current_pose_sub_ = nh.subscribe("current_pose", 10, &vehicleDistanceCheck::currentPoseCallback, this);
}

vehicleDistanceCheck::~vehicleDistanceCheck()
{
}

void vehicleDistanceCheck::safetyWaypointCallback(const autoware_msgs::LaneConstPtr& msg)
{
    if (safety_current_pose_.id != msg->waypoints[0].wpc.waypoint_id)
    {
        safety_previous_pose_.x = safety_current_pose_.x;
        safety_previous_pose_.y = safety_current_pose_.y;
        safety_previous_pose_.id = safety_current_pose_.id;
    }

    safety_current_pose_.x = msg->waypoints[0].pose.pose.position.x;
    safety_current_pose_.y = msg->waypoints[0].pose.pose.position.y;
    safety_current_pose_.id = msg->waypoints[0].wpc.waypoint_id;
    vehicle_waypoint_distance_ = msg->waypoints[0].wpc.vehicle_waypoint_distance;
}

void vehicleDistanceCheck::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    current_pose_x_ = msg->pose.position.x;
    current_pose_y_ = msg->pose.position.y;
}

double vehicleDistanceCheck::verticalDistance(const double& current_pose_x, const double& current_pose_y)
{
    // safety_waypointを受け取れていない, previousが0 の場合は0で返す 
    if ((safety_current_pose_.x == 0 && safety_current_pose_.y == 0) || 
        (safety_previous_pose_.x == 0 && safety_previous_pose_.y == 0)) 
    {
        return 0.0;
    }

    // 傾き0のとき
    if (safety_current_pose_.x == safety_previous_pose_.x)
    {
        return std::abs(current_pose_x - safety_previous_pose_.x);
    }

    if (safety_current_pose_.y == safety_previous_pose_.y)
    {
        return std::abs(current_pose_y - safety_previous_pose_.x);
    }

    double a = (safety_previous_pose_.y - safety_current_pose_.y) / (safety_previous_pose_.x - safety_current_pose_.x);
    double b = -1.0;
    double c = safety_current_pose_.y - a * safety_current_pose_.x;

    vehicle_msg_.a = a;
    vehicle_msg_.c = c;
    vehicle_msg_.current.position.x = current_pose_x_;
    vehicle_msg_.current.position.y = current_pose_y_;
    // d = |a * x + b * y + c| / sqrt(a ^ 2 + b ^ 2)
    return std::abs(a * current_pose_x + b * current_pose_y + c) / std::sqrt(a * a + 1);
}

void vehicleDistanceCheck::vehicleDistanceState()
{
    double d = verticalDistance(current_pose_x_, current_pose_y_);

    vehicle_msg_.safety_current.position.x = safety_current_pose_.x; 
    vehicle_msg_.safety_current.position.y = safety_current_pose_.y; 
    vehicle_msg_.safety_current.id = safety_current_pose_.id; 
    vehicle_msg_.safety_previous.position.x = safety_previous_pose_.x; 
    vehicle_msg_.safety_previous.position.y = safety_previous_pose_.y; 
    vehicle_msg_.safety_previous.id = safety_previous_pose_.id; 
    vehicle_msg_.vehicle_distance = d;
    vehicle_distance_pub_.publish(vehicle_msg_);

    if (vehicle_waypoint_distance_ < d)
    {
        autoware_msgs::RemoteCmd remote_cmd;
        remote_cmd.vehicle_cmd.emergency = true;
        // 経路外判定であることをlost_localizationへ伝える
        remote_cmd.control_mode = 3;
        remote_cmd_pub_.publish(remote_cmd);
    }
}

void vehicleDistanceCheck::run()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        vehicleDistanceState();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_waypoint_distance_checker");
    vehicleDistanceCheck vehicle_distance_check;
    vehicle_distance_check.run();
    return 0;
}
