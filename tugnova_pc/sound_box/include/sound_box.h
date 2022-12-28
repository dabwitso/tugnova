// controls value of plc flag responsible for playing specific task sounds.
#ifndef SOUND_BOX_H_
#define SOUND_BOX_H_

#include <autoware_msgs/LaneArray.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <udp_msgs/UdpControlPacket.h>
#include <udp_msgs/UdpSensorPacket.h>

namespace sound_ns{
enum VoiceSounds {
    WARNING_SOUND = 1 ,
    START_SOUND = 2,
    MOVING_SOUND = 3,
    WINKER_RIGHT_SOUND = 4,
    WINKER_LEFT_SOUND = 5,
    CROSS_ROAD_SOUND = 6,
    x2DLIDAR_DETECTION = 7,
    IDLE_SOUND = 8,
    BEFORE_AUTO_CARGO_LOAD_SOUND = 9,
    AUTO_CARGO_LOADING_SOUND = 10,
    AUTO_CARGO_LOADING_FINISH_SOUND = 11,
    OFFLOAD_CARGO_REQUEST_SOUND = 12,
    ARRIVAL_SOUND = 13,
    LIVOX_DETECTION_SOUND = 14,
  };
class SoundBox {
  public:
    SoundBox();
    void run();
    void publishMsg();


  private:
  ros::NodeHandle nh;

  ros::Publisher plc_packet_pub;

  ros::Subscriber plc_converter_sub;
  ros::Subscriber plc_sensor_packet_sub;
  ros::Subscriber hmi_server_sub;
  ros::Subscriber waypoint_sub;
  ros::Subscriber livox_sub;

  //int counter;
  //int failure_counter;
  int sound_flag;
  int winker_flag;
  int cross_point_flag;
  int detect_2Dlidar;
  int cargo_state;
  int idle_counter;

  bool isCrossRoad;
  bool isLivoxDetectionPt;

  static const int ON = 1;
  static const int OFF = 0;
  static const int INIT = 0;
  static const int NOW = 0;
  static const int PUBLISH_RATE = 100; //100Hz
  static const int ERROR_2DLIDAR_1 = 4;
  static const int ERROR_2DLIDAR_2 = 5;
  static const int LIVOX_ON = 1;
  static const int LIVOX_OFF = 2;


  // default tone sound flags
  static const int IDLE = 0;
  static const int FAILURE = 1;
  static const int START = 2;
  static const int MOVING = 3;
  static const int ARRIVAL = 4;

  static const int IDLE_SOUND_CYCLE = 6000; // 6000 cycles = 1 min as publishing at 100Hz
  static const int LEFT_WINKER = 1;
  static const int RIGHT_WINKER = 2;
  static const int START_LOADING_CARGO = 1;
  static const int LOADING_CARGO = 2;
  static const int LOADING_CARGO_COMPLETE = 3;
  static const int OFFLOAD_CARGO_REQUEST = 4;

  const double HEART_BEAT_THRESHOLD = 0.015; // in secs
  const double INITD = 0.0;

  double plc_converter_time;


  udp_msgs::UdpControlPacket udp_control_packet_msg;
  udp_msgs::UdpSensorPacket udp_sensor_packet_msg;

  VoiceSounds sound_type;

  // methods
  void cal_state();

  // callbacks
  void plcConverterCallback(const udp_msgs::UdpControlPacket& msg);
  void plcSensorPacketCallback(const udp_msgs::UdpSensorPacket& msg);
  void hmiServerCallback(const std_msgs::Int16& msg);
  void waypointsCallback(const autoware_msgs::Lane& msg);
  void livoxCallback(const std_msgs::Int16& msg);
};
}

#endif
