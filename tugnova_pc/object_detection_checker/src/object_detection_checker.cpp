#include "object_detection_checker.h"

ObjectChecker::ObjectChecker()
{
  // ros handles
  // publication
  stopflg_pub = nh.advertise<std_msgs::Int16>("/stop_drive", 1);
  startflg_pub = nh.advertise<std_msgs::Int16>("/start_drive", 1);
  livox_sound_pub = nh.advertise<std_msgs::Int16>("/livox_sound_flg", 10);

  // subscription
  reset_sub = nh.subscribe("plc_sensor_packet", 10, &ObjectChecker::resetCallback, this);
  detection_zone_flg_sub = nh.subscribe("/safety_waypoints", 5, &ObjectChecker::detectionZoneFlgCallback, this);
  livox_sub = nh.subscribe("/livox_state", 10, &ObjectChecker::livoxCallback, this);
  lidar_2d_sub = nh.subscribe("/plc_sensor_packet", 1, &ObjectChecker::lidar2DCallback, this);
  lidar_3d_sub = nh.subscribe("/send_error_status", 1, &ObjectChecker::lidar3DCallback, this);
  second_stop_sub = nh.subscribe("/second_stop_flg", 1, &ObjectChecker::secondStopCallback, this);
  livox_health_sub = nh.subscribe("/livox_health", 1, &ObjectChecker::livoxHealthCallback, this);

  // variable initialization
  isSecondStop = false;
  isDetectionZone = false;
  isDetectionPoint = false;
  isLivoxHealthy = true;
  livox_detected = ON;
  lidar_2D_detected = OFF;
  lidar_3D_detected = OFF;
  waypoint_id_lock = OFF;
  previous_drive_mode_state = OFF;

  OBJECT_ON_2D_LIDAR = { 4, 5 };
}

void ObjectChecker::livoxHealthCallback(const std_msgs::Bool& msg)
{
  isLivoxHealthy = msg.data;
}

void ObjectChecker::resetCallback(const udp_msgs::UdpSensorPacket& msg)
{
  // toggle reset when drive mode selector switch
  // is change from manual to auto and vise versa
  if (msg.ECUMode != previous_drive_mode_state) {
    ROS_INFO("Resetting all parameters...");
    isSecondStop = false;
    isStopped = false;
    isDetectionZone = false;
    isDetectionPoint = false;
    isLivoxHealthy = true;
    livox_detected = ON;
    lidar_2D_detected = OFF;
    lidar_3D_detected = OFF;
    waypoint_id_lock = OFF;
    error_display_count = INIT;

    previous_drive_mode_state = msg.ECUMode;
    std_msgs::Int16 set_sound_msg;
    set_sound_msg.data = LIVOX_OFF;
    livox_sound_pub.publish(set_sound_msg);
  }
}

void ObjectChecker::livoxCallback(const std_msgs::Int16& msg)
{
  if (msg.data) {
    ROS_INFO("livox: false");
    livox_detected = ON;
  } else {
    ROS_INFO("livox: true");
    livox_detected = OFF;
  }
}

void ObjectChecker::lidar3DCallback(const std_msgs::String& msg)
{
  if (msg.data == OBJECT_ON_3D_LIDAR) {
    lidar_3D_detected = ON;
    ROS_INFO("3D lidar on");
  } else {
    lidar_3D_detected = OFF;
  }
}

void ObjectChecker::lidar2DCallback(const udp_msgs::UdpSensorPacket& msg)
{
  if (std::find(OBJECT_ON_2D_LIDAR.begin(), OBJECT_ON_2D_LIDAR.end(), msg.BrakePotVol)
      != OBJECT_ON_2D_LIDAR.end()) {
    lidar_2D_detected = ON;
    ROS_INFO("2D lidar on");
  } else {
    lidar_2D_detected = OFF;
  }
}

void ObjectChecker::secondStopCallback(const std_msgs::Int16& msg)
{
  if (msg.data)
    ROS_INFO("livox second stop: true");
  isSecondStop = true;
}

void ObjectChecker::decisionMaker()
{
  std_msgs::Int16 set_sound_msg;
  if (isDetectionPoint) {
    std_msgs::Int16 drive_cmd;
    drive_cmd.data = ON;

    if (isLivoxHealthy) {
      error_display_count = INIT;
      if (isDetectionZone) {
        // same as "if (lidar_2D_detected || lidar_2D_detected || livox_detected)"
        // written explicitly for easier comprehension purpose only
        if (lidar_2D_detected == ON || lidar_3D_detected == ON || livox_detected == ON) {
          // send stop drive command
          if (!isStopped) {
            stopflg_pub.publish(drive_cmd);
            isStopped = true;
            ROS_INFO("Stop signal initialized");
            set_sound_msg.data = LIVOX_ON;
          }
        } else {
          // send start drive command
          isDetectionPoint = false;
          isStopped = false;
          startflg_pub.publish(drive_cmd);
          ROS_INFO("Start signal initialized");
          set_sound_msg.data = LIVOX_OFF;
        }
      } else {
        // rare case of stop transition from detection area to non detection area
        isDetectionPoint = false;
        isStopped = false;
        startflg_pub.publish(drive_cmd);
        ROS_INFO("Start signal initialized at detection zone end point");
        set_sound_msg.data = LIVOX_OFF;
      }
    } else {
      // turn off livox detection in progress sound
      set_sound_msg.data = LIVOX_OFF;
      // continuously publish stop_drive until livox comes back online
      stopflg_pub.publish(drive_cmd);
      // display error notice to terminal
      if (error_display_count == INIT) {
        system(("bash " + SPAWN_ERROR_TERMINAL).c_str());
        ++error_display_count;
      }
    }
  }
  livox_sound_pub.publish(set_sound_msg);
}

void ObjectChecker::detectionZoneFlgCallback(const autoware_msgs::Lane& msg)
{
  int OD_flag = msg.waypoints[NOW].wpc.object_detection_intersect;
  if (OD_flag == DETECTION_END_FLAG) {
    // exited the object detection zone
    ROS_INFO("End of detection zone");
    isDetectionZone = false;
    waypoint_id_lock = OFF;
    livox_detected = ON;
  } else if (waypoint_id_lock == OFF && OD_flag != OFF && OD_flag != SECOND_STOP_FLAG && OD_flag != DETECTION_END_FLAG) {
    // trigger at waypoints with object_detection_intersect flag only
    ROS_INFO("Triggered object detection flag point stop");
    isDetectionPoint = true;
    waypoint_id_lock = ON;
    isDetectionZone = true;
  } else if (waypoint_id_lock == ON && OD_flag == SECOND_STOP_FLAG && isSecondStop) {
    // trigger at special secondary stop point only
    ROS_INFO("Triggered second stop");
    isDetectionPoint = true;
    isSecondStop = false;
    livox_detected = ON;
    waypoint_id_lock = OFF;
  } else if (lidar_2D_detected == ON || lidar_3D_detected == ON) {
    // trigger within detection zone only when lidar sensors trigger stop
    if (isDetectionZone && !isDetectionPoint) {
      ROS_INFO("Triggered stop by 2D or 3D lidar detection");
      livox_detected = ON;
      isDetectionPoint = true;
    }
  }

  // check if all detection sensors for a go ahead
  decisionMaker();
}

void ObjectChecker::run()
{
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_checker");
  ObjectChecker detection_check_list;
  detection_check_list.run();

  return 0;
}
