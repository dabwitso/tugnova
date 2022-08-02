#include "battery_meter.h"

BatteryMeter::BatteryMeter()
    : private_nh("~")
{
  // ros handles
  // publication
  battery_percentage_pub = nh.advertise<std_msgs::Float64>("battery_percentage", 10);
  battery_level_pub = nh.advertise<std_msgs::String>("battery_range", 10);

  // subscription
  plc_sensor_sub = nh.subscribe("plc_sensor_packet", 10, &BatteryMeter::plcSensorCallback, this);
  battery_low_threshold_sub = nh.subscribe("battery_threshold", 10, &BatteryMeter::batteryThresholdCallback, this);

  // variable initialization
  battery_percentage = 0.0;
}

void BatteryMeter::plcSensorCallback(const udp_msgs::UdpSensorPacket& msg)
{
  double current_voltage = msg.Btry_V;
  std_msgs::Float64 percent;
  if (current_voltage >= MAX_VOLTAGE) {
    battery_percentage = 100.0;
    percent.data = battery_percentage;
    battery_percentage_pub.publish(percent);
  } else if (current_voltage <= MIN_VOLTAGE) {
    battery_percentage = 0.0;
    percent.data = battery_percentage;
    battery_percentage_pub.publish(percent);
  } else {
    battery_percentage = ((current_voltage - MIN_VOLTAGE) / RANGE) * 100;
    percent.data = battery_percentage;
    battery_percentage_pub.publish(percent);
  }
}

void BatteryMeter::batteryThresholdCallback(const std_msgs::Float64& msg)
{
  double battery_threshold = msg.data;
  std_msgs::String state;
  if (battery_percentage <= battery_threshold) {
    state.data = BATTERY_LEVEL[0];
    battery_level_pub.publish(state);
  } else if (battery_percentage > battery_threshold && battery_percentage <= 50.0) {
    state.data = BATTERY_LEVEL[1];
    battery_level_pub.publish(state);
  } else {
    state.data = BATTERY_LEVEL[2];
    battery_level_pub.publish(state);
  }
}

void BatteryMeter::run()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_meter");
  BatteryMeter battery;
  battery.run();
  return 0;
}
