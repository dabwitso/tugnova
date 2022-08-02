#include <ros/ros.h>
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

double hotspot_x = 0.0f;
double hotspot_y = 0.0f;
double hotspot_z = 0.0f;
double hotspot_yaw = 0.0f;
double distance = 0.0f;
double angle = 0.0f;


/** current_poseの受信. */
void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  tf::Vector3 v1(hotspot_x, hotspot_y, hotspot_z);
  tf::Vector3 v2(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  double roll;
  double pitch;
  double yaw;
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  if (distance < tf::tfDistance(v1, v2)) {
    std::cout << "NG - DISTANCE" << std::endl;
  } else if (angle < std::abs(yaw - hotspot_yaw)) {
    std::cout << "NG - ANGLE" << std::endl;
  } else {
    std::cout << "OK" << std::endl;
  }
  exit(0);
}

/** メイン. */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hotspot_position_check");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("x", hotspot_x);
  private_nh.getParam("y", hotspot_y);
  private_nh.getParam("z", hotspot_z);
  private_nh.getParam("yaw", hotspot_yaw);
  private_nh.getParam("distance", distance);
  private_nh.getParam("angle", angle);

  ros::Subscriber sub_current_pose = n.subscribe("/current_pose", 10, currentPoseCallback);

  ros::spin();

  return 0;
}
