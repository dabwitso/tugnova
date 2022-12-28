#ifndef OBJECT_DETECTION_CHECKER
#define OBJECT_DETECTION_CHECKER

#include <algorithm>
#include <array>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <string>
#include <udp_msgs/UdpSensorPacket.h>

class ObjectChecker {
  public:
  ObjectChecker();
  void run();

  private:
  ros::NodeHandle nh;

  ros::Publisher stopflg_pub;
  ros::Publisher startflg_pub;
  ros::Publisher detection_sound_pub;
  ros::Publisher livox_sound_pub;

  ros::Subscriber reset_sub;
  ros::Subscriber detection_zone_flg_sub;
  ros::Subscriber livox_sub;
  ros::Subscriber lidar_2d_sub;
  ros::Subscriber lidar_3d_sub;
  ros::Subscriber second_stop_sub;
  ros::Subscriber livox_health_sub;

  int livox_detected;
  int lidar_2D_detected;
  int lidar_3D_detected;
  int waypoint_id_lock;
  int previous_drive_mode_state;
  int error_display_count;

  static const int ON = 1;
  static const int OFF = 0;
  static const int NOW = 0;
  static const int INIT = 0;
  static const int SECOND_STOP_FLAG = 100;
  static const int DETECTION_END_FLAG = 200;
  static const int LIVOX_ON = 1;
  static const int LIVOX_OFF = 2;

  bool isSecondStop;
  bool isStopped;
  bool isDetectionZone;
  bool isDetectionPoint;
  bool isLivoxHealthy;

  const std::string OBJECT_ON_3D_LIDAR = "velocity_set_1_0";
  const std::string SPAWN_ERROR_TERMINAL = "/home/nvidia/Autoware/ros/nodeshell/spawn_livox_script.sh";

  std::array<int, 2> OBJECT_ON_2D_LIDAR;

  // misc
  void decisionMaker();

  // callback functions
  void livoxCallback(const std_msgs::Int16& msg);
  void detectionZoneFlgCallback(const autoware_msgs::Lane& msg);
  void secondStopCallback(const std_msgs::Int16& msg);
  void lidar3DCallback(const std_msgs::String& msg);
  void lidar2DCallback(const udp_msgs::UdpSensorPacket& msg);
  void resetCallback(const udp_msgs::UdpSensorPacket& msg);
  void livoxHealthCallback(const std_msgs::Bool& msg);
};

#endif
