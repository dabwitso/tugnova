#include "ros/ros.h"
#include "stdlib.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sstream>

int main(int argc, char *argv[])
{
  // args: x y z roll pitch yaw
  if (argc != 7)
  {
    ROS_ERROR("Cannot Publish Initialpose.");
    return 1;
  }

  ros::init(argc, argv, "pub_initialpose");
  ros::NodeHandle n;
  ros::Publisher ctrl_car_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
  
  ros::Rate loop_rate(10);

  geometry_msgs::PoseWithCovarianceStamped msg;
  std_msgs::Header header;
  geometry_msgs::PoseWithCovariance pose_with_covariance;
  geometry_msgs::Pose pose;
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;

  header.frame_id = "map";

  position.x = atof(argv[1]);
  position.y = atof(argv[2]);
  position.z = atof(argv[3]);

  tf::Quaternion quat = tf::createQuaternionFromRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
  quaternionTFToMsg(quat, orientation);

  pose.position = position;
  pose.orientation = orientation;
  pose_with_covariance.pose = pose;
  pose_with_covariance.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

  msg.header = header;
  msg.pose = pose_with_covariance;

  ROS_INFO("position.x: %f", position.x);
  ROS_INFO("position.y: %f", position.y);
  ROS_INFO("position.z: %f", position.z);
  ROS_INFO("orientation.x: %f", orientation.x);
  ROS_INFO("orientation.y: %f", orientation.y);
  ROS_INFO("orientation.z: %f", orientation.z);
  ROS_INFO("orientation.w: %f", orientation.w);

  int count = 0;
  while (ros::ok())
  {
    ctrl_car_pub.publish(msg);
    loop_rate.sleep();
    count++;
    if (count >= 30)
    {
      break;
    }
  }

  return 0;
}
