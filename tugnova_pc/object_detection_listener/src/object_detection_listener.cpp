#include "object_detection_listener.h"

ObjectListener::ObjectListener()
    : private_nh("~")
{
  // ros handles
  // publication
  detection_zone_pub = nh.advertise<std_msgs::String>("/start", 1);
  stopflg_pub = nh.advertise<std_msgs::Int16>("/stop_drive", 1);
  startflg_pub = nh.advertise<std_msgs::Int16>("/start_drive", 10);

  // subscription
  detection_result_sub = nh.subscribe("/result", 10, &ObjectListener::detectionResultCallback, this);
  detection_zone_flg_sub = nh.subscribe("/safety_waypoints", 10, &ObjectListener::detectionZoneFlgCallback, this);

  // variable initialization

  message_publish_count = INIT;
  detection_point_id = MAX_WAYPOINT_ID;
  isStopped = false;
}

void ObjectListener::detectionResultCallback(const std_msgs::Bool& msg)
{
  if (!msg.data) {
    ROS_INFO("OK status from object detection node. Starting to drive...");
    std_msgs::Int16 start_drive;
    start_drive.data = ON;
    startflg_pub.publish(start_drive);

    // signal to stop FIR camera processing
    std_msgs::String message;
    message.data = "stop";
    detection_zone_pub.publish(message);
  }
}

void ObjectListener::detectionZoneFlgCallback(const autoware_msgs::Lane& msg)
{
  int intersection_ID = msg.waypoints[NOW].wpc.object_detection_intersect;
  int current_waypoint_id = msg.waypoints[NOW].wpc.waypoint_id;

  if (intersection_ID != OFF) {
    if (message_publish_count < TOTAL_MESSAGE_COUNT_TO_JETSON) {
      message_publish_count++;
      std_msgs::String detection_flg;
      detection_flg.data = "start_intersection" + std::to_string(intersection_ID);
      detection_zone_pub.publish(detection_flg);
      ROS_INFO("Published command %s to Jetson", detection_flg.data);
    }
    if (!isStopped) {
      // send only once to drive_stop_controller node
      ROS_INFO("Stopping car");
      std_msgs::Int16 stop_cmd;
      stop_cmd.data = ON;
      stopflg_pub.publish(stop_cmd);
      isStopped = true;
      detection_point_id = current_waypoint_id;
    }
  }
  // guard against stop trigger when still at same point
  if (current_waypoint_id > detection_point_id) {
    message_publish_count = INIT;
    isStopped = false;
    detection_point_id = MAX_WAYPOINT_ID;
  }
}

void ObjectListener::run()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_listener");
  ObjectListener detector;
  detector.run();

  return 0;
}
