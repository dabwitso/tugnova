#include "pcd2int.h"

Pcd2Int::Pcd2Int()
{
  pcd_status_pub = nh.advertise<std_msgs::Int16>("/pcd_status", 1);

  points_raw_sub = nh.subscribe("/points_raw", 1, &Pcd2Int::pointsRawCallback, this);

  state = OFF;
}

void Pcd2Int::pointsRawCallback(const sensor_msgs::PointCloud2& msg)
{
  state = ON;
}

void Pcd2Int::run()
{
  ros::Rate loop_rate(PUBLISH_RATE);
  while (ros::ok()) {
    std_msgs::Int16 msg;
    msg.data = state;
    pcd_status_pub.publish(msg);

    state = OFF;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_to_int_converter");
  Pcd2Int converter;
  converter.run();

  return 0;
}
