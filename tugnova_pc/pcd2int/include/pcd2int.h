#ifndef PCD2INT_H
#define PCD2INT_H

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Int16.h>

class Pcd2Int{
  public:
    Pcd2Int();
    void run();
  private:
    ros::NodeHandle nh;

    ros::Publisher pcd_status_pub;

    ros::Subscriber points_raw_sub;

    static const int ON = 1;
    static const int OFF = 0;
    static const int PUBLISH_RATE = 10; //10 Hz

    int state;


    void pointsRawCallback(const sensor_msgs::PointCloud2& msg);
};

#endif

