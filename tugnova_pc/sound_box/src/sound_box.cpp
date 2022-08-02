#include "sound_box.h"

namespace sound_ns {
SoundBox::SoundBox()
    : private_nh("~")
{
  // ros handles
  plc_packet_pub = nh.advertise<udp_msgs::UdpControlPacket>("/plc_control_packet", 1);
  plc_converter_sub = nh.subscribe("/plc_control_packet_raw", 1, &SoundBox::plcConverterCallback, this);
  plc_sensor_packet_sub = nh.subscribe("/plc_sensor_packet", 1, &SoundBox::plcSensorPacketCallback, this);
  hmi_server_sub = nh.subscribe("/cargo_task_state", 1, &SoundBox::hmiServerCallback, this);
  waypoint_sub = nh.subscribe("/safety_waypoints", 1, &SoundBox::waypointsCallback, this);

  // variable initialization
  sound_flag = INIT;
  cross_point_flag = INIT;
  idle_counter = INIT;
  winker_flag = INIT;
  detect_2Dlidar = INIT;
  cargo_state = INIT;
  plc_converter_time = INITD;
  isCrossRoad = false;
}
void SoundBox::publishMsg()
{
  plc_packet_pub.publish(udp_control_packet_msg);
}

void SoundBox::run()
{
  ros::Rate loop_rate(PUBLISH_RATE);
  while (ros::ok()) {
    ros::Time loop_time = ros::Time::now();
    double current_time = loop_time.toSec();
    double time_elapsed = current_time - plc_converter_time;
    // check if plc_converter is publishing on topic /plc_control_packet_raw
    if (time_elapsed < HEART_BEAT_THRESHOLD) {
      //ROS_INFO("successful publishing to /plc_control_packet");
      cal_state();
    } else {
      //ROS_ERROR("/plc_control_packet_raw not publishing from plc_converter node");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void SoundBox::cal_state()
{
  sound_flag = udp_control_packet_msg.brk_cmode;
  // failure
  if (sound_flag == FAILURE) {
    idle_counter = INIT;
    cargo_state = INIT;
    if (detect_2Dlidar == ERROR_2DLIDAR_1 || detect_2Dlidar == ERROR_2DLIDAR_2) {
      ROS_INFO("2D obstacle detected");
      // object detected by 2D lidar
      sound_type = x2DLIDAR_DETECTION;
      udp_control_packet_msg.brk_cmode = sound_type;
      publishMsg();
    } else {
      ROS_INFO("General Failure");
      sound_type = WARNING_SOUND;
      udp_control_packet_msg.brk_cmode = sound_type;
      publishMsg();
    }

  } else if (sound_flag == START) {
    ROS_INFO("Start flag");
    cargo_state = INIT;
    idle_counter = INIT;
    sound_type = START_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();

  } else if (sound_flag == MOVING && winker_flag == OFF) {
    ROS_INFO("moving");
    cross_point_flag = INIT;
    sound_type = MOVING_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();

  } else if (winker_flag == LEFT_WINKER) {
    ROS_INFO("winker_left");
    sound_type = WINKER_LEFT_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();

  } else if (winker_flag == RIGHT_WINKER) {
    ROS_INFO("winker_right");
    sound_type = WINKER_RIGHT_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();

  } else if (sound_flag == ARRIVAL && !isCrossRoad) {
    // don't consider a pause point as final destination.
    ROS_INFO("Arrival");
    sound_type = ARRIVAL_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();

  } else if (cargo_state == START_LOADING_CARGO) {
    ROS_INFO("Before loading");
    sound_type = BEFORE_AUTO_CARGO_LOAD_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();

  } else if (cargo_state == LOADING_CARGO) {
    ROS_INFO("Loading");
    sound_type = AUTO_CARGO_LOADING_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();

  } else if (cargo_state == LOADING_CARGO_COMPLETE) {
    ROS_INFO("Loading complete");
    sound_type = AUTO_CARGO_LOADING_FINISH_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();

  } else if (cargo_state == OFFLOAD_CARGO_REQUEST) {
    ROS_INFO("offload request");
    sound_type = OFFLOAD_CARGO_REQUEST_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();
  } else if (sound_flag == IDLE) {
    // play idle sound once very IDLE_SOUND_CYCLE time
    if (idle_counter == IDLE_SOUND_CYCLE) {
      ROS_INFO("Idle sound");
      sound_type = IDLE_SOUND;
      udp_control_packet_msg.brk_cmode = sound_type;
      idle_counter = INIT;
    }
    publishMsg();
    ++idle_counter;
  } else if (cross_point_flag) {
    ROS_INFO("cross_road sound");
    sound_type = CROSS_ROAD_SOUND;
    udp_control_packet_msg.brk_cmode = sound_type;
    publishMsg();
  } else {
    publishMsg();
  }
}

void SoundBox::plcConverterCallback(const udp_msgs::UdpControlPacket& msg)
{
  ros::Time plc_message_time = ros::Time::now();
  plc_converter_time = plc_message_time.toSec();
  udp_control_packet_msg = msg;
}

void SoundBox::plcSensorPacketCallback(const udp_msgs::UdpSensorPacket& msg)
{
  // get 2D lidar plc warning message
  detect_2Dlidar = msg.BrakePotVol;
}

void SoundBox::hmiServerCallback(const std_msgs::Int16& msg)
{
  cargo_state = msg.data;
}

void SoundBox::waypointsCallback(const autoware_msgs::Lane& msg)
{
  switch (msg.waypoints[NOW].wpc.winker_point) {
  case 1:
  case 2:
    winker_flag = msg.waypoints[NOW].wpc.winker_point;
    break;
  default:
    winker_flag = OFF;
  }
  if (msg.waypoints[NOW].wpc.pause_point == ON && winker_flag == OFF) {
    cross_point_flag = ON;
    isCrossRoad = true;
  }else{
     cross_point_flag = OFF;
     isCrossRoad = false;
  }
}

} // namespace sound_ns

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sound_box");
  sound_ns::SoundBox sound_controller;
  sound_controller.run();
}
