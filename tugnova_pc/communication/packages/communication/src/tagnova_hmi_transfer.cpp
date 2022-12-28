#include "tagnova_hmi_transfer.h"

HmiConnect::HmiConnect()
    : private_nh("~")
{
  // ros handles
  // publications
  waitpoint_pub = nh.advertise<std_msgs::Bool>("waitpoint_state", 10);
  stats_pub = nh.advertise<communication_msgs::Stats>("tagnova_stats", 10);
  marionet_pub = nh.advertise<communication_msgs::MarionetteMessage>("marionette_status", 1);
  drive_mode_pub = nh.advertise<std_msgs::Int16>("tugnova_drive_mode", 10);
  //stop_flg_pub = nh.advertise<std_msgs::Int16>("detection_stop_point", 10);

  // subscriptions
  pose_sub = nh.subscribe("current_pose", 1, &HmiConnect::poseCallback, this);
  vehicle_status_sub = nh.subscribe("vehicle_status", 1, &HmiConnect::vehicleStatusCallback, this);
  location_sub = nh.subscribe("vehicle_location", 1, &HmiConnect::locationCallback, this);
  plc_sensor_sub = nh.subscribe("plc_sensor_packet", 10, &HmiConnect::plcSensorCallback, this);

  battery_status_sub = nh.subscribe("battery_status", 10, &HmiConnect::batteryHealthCallback, this);

  battery_percent_sub = nh.subscribe("battery_percentage", 10, &HmiConnect::batteryPercentCallback, this);

  waitpoint_sub = nh.subscribe("safety_waypoints", 1, &HmiConnect::waitpointCallback, this);
  waitpoint_clear_sub = nh.subscribe("waitpoint_clear", 10, &HmiConnect::waitpointClearCallback, this);
  //sub_traffic_light_flg = nh.subscribe("safety_waypoints", 10, &HmiConnect::trafficLightCallback, this);
  vehicle_twist_sub = nh.subscribe("out_twist_cmd", 10, &HmiConnect::twistCallback, this);

  // variable initialization
  //traffic_light_flg = INIT;
  plc_error = INIT;
  waypoint_id = INIT;
  waitpoint_flg = INIT;
  battery_health_id = INIT;
  drive_mode_pub_count = INIT;
  //traffic_flg_publish_count = INIT;

  x_position = DINIT;
  y_position = DINIT;
  z_position = DINIT;
  speed = DINIT;
  battery_percentage = DINIT;

  isVehicleIdle = true;

  battery_health = {
    {0, "normal"},
    {1, "warning"},
    {2, "error"},
    {3, "fatal"},
  };
}

void HmiConnect::driveModePublish(int data){
  if(drive_mode_pub_count > DRIVE_MODE_PUBLISH_CYCLE){
    std_msgs::Int16 msg;
    msg.data = data;
    drive_mode_pub.publish(msg);
    // reset flag
    drive_mode_pub_count = INIT;
  }
  ++drive_mode_pub_count;
}

void HmiConnect::publishStats()
{
  communication_msgs::Stats msg;
  msg.waypoint_id = waypoint_id;
  msg.position.x = x_position;
  msg.position.y = y_position;
  msg.position.z = z_position;
  msg.speed = speed;
  msg.plc_error = plc_error;
  msg.battery_info = battery_health[battery_health_id];
  msg.battery_status = battery_percentage;
  stats_pub.publish(msg);

  communication_msgs::MarionetteMessage marionet_msg;
  marionet_msg.plc_status = plc_error;
  marionet_msg.battery = battery_percentage;
  marionet_pub.publish(marionet_msg);
}

void HmiConnect::plcSensorCallback(const udp_msgs::UdpSensorPacket& msg)
{
  plc_error = msg.BrakePotVol;
  driveModePublish(msg.ECUMode);
}

void HmiConnect::twistCallback(const geometry_msgs::TwistStamped& msg)
{
  if (msg.twist.linear.x == ZERO_SPEED) {
    isVehicleIdle = true;
  } else {
    isVehicleIdle = false;
  }
}

void HmiConnect::batteryPercentCallback(const std_msgs::Float64& msg)
{
  // only update battery percentage when vehicle is stationary
  if (isVehicleIdle) {
    battery_percentage = msg.data;
  }
}

void HmiConnect::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  x_position = msg->pose.position.x;
  y_position = msg->pose.position.y;
  z_position = msg->pose.position.z;
}

void HmiConnect::vehicleStatusCallback(const autoware_msgs::VehicleStatusConstPtr& msg)
{
  speed = msg->speed;
}

void HmiConnect::locationCallback(const autoware_msgs::VehicleLocationConstPtr& msg)
{
  waypoint_id = msg->waypoint_index;
}

void HmiConnect::waitpointClearCallback(const std_msgs::String& msg)
{
  ROS_INFO("clearing waitpoint");
  if (msg.data == "clear") {
    waitpoint_flg = OFF;
  }
}

void HmiConnect::waitpointCallback(const autoware_msgs::Lane& msg)
{
  if (msg.waypoints[NOW].wpc.waiting_state == ON) {
    if (waitpoint_flg == OFF) {
      std_msgs::Bool message;
      message.data = true;
      waitpoint_pub.publish(message);
      waitpoint_flg = ON;
    }
  } else {
    std_msgs::Bool message;
    message.data = false;
    waitpoint_pub.publish(message);
  }
}

//void HmiConnect::trafficLightCallback(const autoware_msgs::Lane& msg)
//{
//  int trl_flag = msg.waypoints[NOW].wpc.traffic_light_flg;
//
//  if (trl_flag == OFF)
//    traffic_flg_publish_count = OFF;
//  else {
//    if (traffic_flg_publish_count == OFF) {
//      ROS_INFO("traffic light flag: %d   traffic_flg_publish_count: %d", trl_flag, traffic_flg_publish_count);
//      if (*(std::find(TRAFFIC_LIGHT_STOP_FLAG.begin(), TRAFFIC_LIGHT_STOP_FLAG.end(), trl_flag)) == trl_flag) {
//        // stop drive if at traffic light stop point
//        ROS_INFO("Stopping tagnova...");
//        std_msgs::Int16 stop_cmd;
//        stop_cmd.data = ON;
//        stop_flg_pub.publish(stop_cmd);
//      }
//      std_msgs::Int16 traffic_msg;
//      traffic_msg.data = trl_flag;
//      traffic_light_flg_pub.publish(trl_flag);
//      ++traffic_flg_publish_count;
//    }
//  }
//}

void HmiConnect::batteryHealthCallback(const carctl_msgs::battery_status& msg)
{
  battery_health_id = msg.color;
}

void HmiConnect::run()
{
  ros::Rate loop_rate(PUBLISH_RATE);
  while (ros::ok()) {
    publishStats();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tagnova_hmi_transfer");
  HmiConnect tagnova_hmi_connect;
  tagnova_hmi_connect.run();
  return 0;
}
