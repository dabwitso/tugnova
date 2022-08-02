#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/console.h>

ros::Publisher vehicleTwistPub;

void vehicleOdometryCallback(const nav_msgs::Odometry &odmMsg)
{
    geometry_msgs::TwistStamped vehicleTwistMsg;
    std_msgs::Header header;

    header.stamp = ros::Time::now();
    header.frame_id = "base_link";

    ROS_DEBUG("vehicleOdometryCallback: linear.x[%g]", odmMsg.twist.twist.linear.x);
    ROS_DEBUG("vehicleOdometryCallback: angular.z[%g]", odmMsg.twist.twist.angular.z);

    // 必要最小限度の情報のみ生成
    vehicleTwistMsg.header = header;
    vehicleTwistMsg.twist.linear.x = odmMsg.twist.twist.linear.x;
    vehicleTwistMsg.twist.angular.z = odmMsg.twist.twist.angular.z;

    vehicleTwistPub.publish(vehicleTwistMsg);
    ros::spinOnce();
}


int main(int argc, char **argv)
{
    ROS_DEBUG("generate_vehicle_twist start");
    ros::init(argc, argv, "generate_vehicle_twist");

    ros::NodeHandle n;
    ros::Subscriber vehicleOdometry = n.subscribe("/vehicle/odom", 10, vehicleOdometryCallback);
    vehicleTwistPub = n.advertise<geometry_msgs::TwistStamped>("/vehicle/twist", 10);

    ros::spin();

    ROS_DEBUG("generate_vehicle_twist end");

    return 0;
}