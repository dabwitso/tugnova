#include "shutter_controller.h"

ShutterController::ShutterController()
    : private_nh("~")
{
  // ros handles
  // publication
  bluetooth_connect_req_pub = nh.advertise<std_msgs::Int8>("shutter_point", 1);
  passed_through_shutter_pub = nh.advertise<std_msgs::Int8>("shutter_passed", 1);
  shutter_stoppoint_stopflg_pub = nh.advertise<std_msgs::Int16>("shutter_stop_flag", 10);
  shutter_resume_drive_pub = nh.advertise<std_msgs::Int16>("/start_drive", 10);
  shutter_error_pub = nh.advertise<std_msgs::Int16>("shutter_error", 10);

  // subscription
  shutter_response_sub = nh.subscribe("shutter_status", 1, &ShutterController::shutterResponseCallback, this);
  bluetooth_connect_sub = nh.subscribe("bluetooth_connection", 1, &ShutterController::shutterBluetoothCallback, this);
  shutter_waypoint_flg_sub = nh.subscribe("safety_waypoints", 10, &ShutterController::shutterFlgsCallback, this);
  clear_waitpoint_sub = nh.subscribe("waitpoint_clear", 10, &ShutterController::clearWaitpointCallback, this);
  gpiostartflg_sub = nh.subscribe("/GpioStartFlg", 10, &ShutterController::gpioStartFlgCallback, this);

  // variable initialization
  shutterStopCount = INIT;
  bluetooth_flg = INIT;
  error = OFF;

  isShutterOpen = false;
  isWaitingBTConnection = false;
}

void ShutterController::shutterStopHandler()
{
  std_msgs::Int16 msg;
  msg.data = ON;
  shutter_stoppoint_stopflg_pub.publish(msg);
  ROS_INFO("At shutter_stop point. Stop command published due to bluetooth control failure");
  shutterStopCount = INIT;
}

void ShutterController::shutterFlgsCallback(const autoware_msgs::Lane& msg)
{
  if (msg.waypoints[NOW].wpc.shutter_point == ON) {
    std_msgs::Int8 message;
    message.data = ON;
    bluetooth_connect_req_pub.publish(message);
    ROS_INFO("Shutter point reached. Bluetooth connection request sent");
  }
  if (msg.waypoints[NOW].wpc.shutter_passed == ON) {
    // shutter passed through
    std_msgs::Int8 message;
    message.data = ON;
    passed_through_shutter_pub.publish(message);
    ROS_INFO("Shutter_passed signal sent to shutter");
    // reset shutter isShutterOpen flag after passing through shutter.
    isShutterOpen = false;
    error = OFF;
  }
  // check if shutter is open to decide whether to stop at shutter_stop position
  // if(msg.waypoints[1].wpc.shutter_stop == 1){} //check if can preempt
  if (msg.waypoints[NOW].wpc.shutter_stop == ON) {
    if (error == OFF) {
      if (bluetooth_flg == connection_state.CONNECTION_ERROR || bluetooth_flg == connection_state.CONNECTION_TIMEOUT) {
        ROS_WARN("Bluetooth connection failure!");
        error = ON;
        std_msgs::Int16 error_msg;
        error_msg.data = error;
        shutter_error_pub.publish(error_msg);
      }
    }

    if (!isShutterOpen) {
      // stop at shutter_stop point if shutter isShutterOpen is false
      ROS_WARN("shutter not open yet!");
      if (bluetooth_flg == connection_state.CONNECTION_SUCCESS) {
        ROS_INFO("Bluetooth connection established. Waiting for shutter to open...");
      } else if (bluetooth_flg == connection_state.CONNECTION_RETRY) {
        ROS_INFO("Bluetooth connection retry...");
      }
      if (shutterStopCount == INIT) {
        ROS_INFO("Stopping at shutter_stop point. Shutter not yet open...");
        shutterStopHandler();
        ++shutterStopCount;
        isWaitingBTConnection = true;
      }
    }
  }
}

void ShutterController::shutterResponseCallback(const std_msgs::String& msg)
{
  // make decision to bypass shutter_stop
  if (msg.data == "open") {
    ROS_INFO("Shutter open");
    isShutterOpen = true;
    if (isWaitingBTConnection) {
      ROS_INFO("Resuming drive");
      std_msgs::Int16 start;
      start.data = ON;
      shutter_resume_drive_pub.publish(start);

      shutterStopCount = INIT;
      isWaitingBTConnection = false;
    }
  } else if (msg.data == "failure") {
    ROS_INFO("Shutter failed to open via bluetooth connection");
    isShutterOpen = false;
    error = ON;
    std_msgs::Int16 error_msg;
    error_msg.data = error;
    shutter_error_pub.publish(error_msg);
  }
}

void ShutterController::clearWaitpointCallback(const std_msgs::String& msg)
{
  if (msg.data == "start") {
    ROS_INFO("Resuming driving by Freedom command");
    std_msgs::Int16 start;
    start.data = ON;
    shutter_resume_drive_pub.publish(start);
    ros::Duration sleep(7);
    error = OFF;

    shutterStopCount = INIT;
    isWaitingBTConnection = false;
  }
}

void ShutterController::gpioStartFlgCallback(const std_msgs::Int16& msg)
{
  if (msg.data == ON) {
    // reset
    shutterStopCount = INIT;
    isWaitingBTConnection = false;
  }
}

void ShutterController::shutterBluetoothCallback(const std_msgs::Int8& msg)
{
  bluetooth_flg = msg.data;
}

void ShutterController::run()
{
  ROS_INFO("shutter controller online...");
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shutter_controller");
  ShutterController shutter_controller;
  shutter_controller.run();

  return 0;
}
