#ifndef CSV_REFRESH_CLOCK_H
#define CSV_REFRESH_CLOCK_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>

class CSV_CLOCK {
  public:
  CSV_CLOCK();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher refresh_csv_pub;
  // 1/60secs = 0.017 rate. i.e, READ_CSV_REFRESH_RATE = 1/desired_refresh_time_in_secs.
  // This default of 0.017 -> refresh data from csv every minute.
  const double READ_CSV_REFRESH_RATE = 0.017;

};
#endif
