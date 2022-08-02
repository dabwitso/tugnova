#ifndef TAGNOVA_HMI_TRANSFER_H
#define TAGNOVA_HMI_TRANSFER_H

#include <algorithm>
#include <array>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleLocation.h>
#include <autoware_msgs/VehicleStatus.h>
#include <carctl_msgs/battery_status.h>
#include <communication_msgs/MarionetteMessage.h>
#include <communication_msgs/Stats.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <map>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdint.h>
#include <string>
#include <udp_msgs/UdpSensorPacket.h>

class HmiConnect {
  public:
  HmiConnect();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher stats_pub;
  ros::Publisher waitpoint_pub;
  ros::Publisher marionet_pub;
  ros::Publisher stop_flg_pub;
  //ros::Publisher traffic_light_flg_pub;

  ros::Subscriber pose_sub;
  ros::Subscriber vehicle_status_sub;
  ros::Subscriber location_sub;
  ros::Subscriber plc_sensor_sub;
  ros::Subscriber battery_status_sub;
  ros::Subscriber battery_percent_sub;
  ros::Subscriber waitpoint_sub;
  ros::Subscriber waitpoint_clear_sub;
  //ros::Subscriber traffic_light_sub;
  ros::Subscriber vehicle_twist_sub;

  //int traffic_light_flg;
  int plc_error;
  int waypoint_id;
  int waitpoint_flg;
  int battery_health_id;
  //int traffic_flg_publish_count;

  double x_position;
  double y_position;
  double z_position;
  double speed;
  double battery_percentage;

  static const int16_t ON = 1;
  static const int16_t OFF = 0;
  static const int16_t INIT = 0;
  static const int16_t NOW = 0;

  static const int16_t PUBLISH_RATE = 10;

  const double DINIT = 0.0;
  const double ZERO_SPEED = 0.0;

  // used to update battery percentage only when vehicle is idle
  bool isVehicleIdle;

  // TRAFFIC_LIGHT_STOP_FLAG are the first value of a pair of flags assigned to a traffic light area
  // example traffic light zone pairs: (1,2),(3,4),(5,6)
  const std::array<int, 3> TRAFFIC_LIGHT_STOP_FLAG = { 1, 3, 5 };

  const std::map<int, std::string> BATTERY_HEALTH = {
    { 0, "normal" },
    { 1, "warning" },
    { 2, "error" },
    { 3, "fatal" },
  };

  // misc functions
  void publishStats();

  // callback functions
  void plcSensorCallback(const udp_msgs::UdpSensorPacket& msg);
  void batteryPercentCallback(const std_msgs::Float64& msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void vehicleStatusCallback(const autoware_msgs::VehicleStatusConstPtr& msg);
  void locationCallback(const autoware_msgs::VehicleLocationConstPtr& msg);
  void waitpointClearCallback(const std_msgs::String& msg);
  void waitpointCallback(const autoware_msgs::Lane& msg);
  //void trafficLightCallback(const autoware_msgs::Lane& msg);
  void batteryHealthCallback(const carctl_msgs::battery_status& msg);
  void twistCallback(const geometry_msgs::TwistStamped& msg);
};

#endif
