#ifndef BATTERY_METER_H
#define BATTERY_METER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <udp_msgs/UdpSensorPacket.h>

class BatteryMeter {
  public:
  BatteryMeter();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher battery_percentage_pub;
  ros::Publisher battery_level_pub;

  ros::Subscriber plc_sensor_sub;
  // this threshold is set in nodeshell/battery_threshold.sh
  ros::Subscriber battery_low_threshold_sub;

  // voltage values set according to manufacturer specifications
  const double MAX_VOLTAGE = 50.0;
  const double MIN_VOLTAGE = 42.0;
  const double RANGE = MAX_VOLTAGE - MIN_VOLTAGE;

  const char* BATTERY_LEVEL[3] = { "low", "medium", "high" };

  double battery_percentage;

  // callback functions

  void plcSensorCallback(const udp_msgs::UdpSensorPacket& msg);
  void batteryThresholdCallback(const std_msgs::Float64& msg);
};

#endif
