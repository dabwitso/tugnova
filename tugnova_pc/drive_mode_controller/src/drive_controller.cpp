#include "drive_controller.h"

DriveController::DriveController()
    : private_nh("~")
{
  // ros handles
  // publication
  twist_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("out_twist_ctrl", 10);
  waitpoint_pub = nh.advertise<std_msgs::Bool>("waitpoint_state", 10);
  magnet_drive_state_pub = nh.advertise<std_msgs::String>("magnet_status", 1);

  // subscription
  plc_sensor_sub = nh.subscribe("plc_sensor_packet", 10, &DriveController::plcSensorCallback, this);
  drive_mode_sub = nh.subscribe("safety_waypoints", 10, &DriveController::driveModeCallback, this);
  gpiostartflg_sub = nh.subscribe("GpioStartFlg", 10, &DriveController::gpioCallback, this);
  direct_stop_node_sub = nh.subscribe("out_twist_raw", 10, &DriveController::inputTwistCallback, this);
  subtask_name_sub = nh.subscribe("route_cmds", 10, &DriveController::subtaskInfoCallback, this);
  reset_flags_sub = nh.subscribe("magnet_drive_reset", 10, &DriveController::resetCallback, this);

  // variable initialization
  isMagnetDrive = false;
  isMagnetStopPoint = false;
  isGpioButtonPressed = false;
  isOnMagnetStrip = false;

  stop_marker_counter = INIT;
  previous_plc_pulse_state = INIT;

  current_route = DEFAULT;
}

void DriveController::readCSV()
{
  // read magnet_count.csv file
  io::CSVReader<2> in(MAGNET_MARKER_LOOKUP_PATH);
  in.read_header(io::ignore_extra_column, "route_id", "stop_position");
  std::string route_id;
  int stop_position;
  while (in.read_row(route_id, stop_position)) {
    stop_marker_lookup[route_id] = stop_position;
  }
}

void DriveController::speedPublisher()
{
  if (isMagnetStopPoint) {
    geometry_msgs::TwistStamped msg;
    msg = direct_stop_twist_cmd;
    msg.twist.linear.x = ZERO_SPEED;
    twist_cmd_pub.publish(msg);
  } else {
    twist_cmd_pub.publish(direct_stop_twist_cmd);
  }
}

void DriveController::plcSensorCallback(const udp_msgs::UdpSensorPacket& msg)
{

  int dict_stop_point = INT_MAX;
  if (isMagnetDrive) {
    // Magnet drive mode
    if (isMagnetStopPoint) {
      speedPublisher();
    } else {
      int current_stop_pt = msg.AccelPotVol;
      if (current_stop_pt != previous_plc_pulse_state && current_stop_pt == 1) {
        ++stop_marker_counter;
        previous_plc_pulse_state = current_stop_pt;
      } else
        previous_plc_pulse_state = current_stop_pt;

      // check if stop point reached

      if (current_route != DEFAULT) {
        dict_stop_point = stop_marker_lookup[current_route];
      }
      if (stop_marker_counter == dict_stop_point) {
        if (!isGpioButtonPressed) {
          ROS_INFO("Magnet stop point reached. Waiting for waitpoint clear command");
          isMagnetStopPoint = true;
          std_msgs::String message;
          message.data = "stop";
          magnet_drive_state_pub.publish(message);
        }
      }
      speedPublisher();
    }

  } else {
    // Autoware drive mode
    isMagnetStopPoint = false;
    stop_marker_counter = INIT;
    previous_plc_pulse_state = INIT;
    isGpioButtonPressed = false;
    speedPublisher();
  }
}

void DriveController::driveModeCallback(const autoware_msgs::Lane& msg)
{
  // selects between Autoware drive and Magnet drive mode
  int mode_change = msg.waypoints[NOW].wpc.mode_change;
  if (mode_change == MAGNET_DRIVE_MODE) {
    isMagnetDrive = true;
    if (!isOnMagnetStrip) {
      std_msgs::String message;
      message.data = "start";
      magnet_drive_state_pub.publish(message);
      isOnMagnetStrip = true;
    }
  } else {
    isMagnetDrive = false;
    isOnMagnetStrip = false;
  }
}

void DriveController::resetCallback(const std_msgs::Int16& msg)
{
  // reset option from command line publication to topic /magnet_drive_reset
  if (msg.data == ON) {
    isMagnetDrive = false;
    isMagnetStopPoint = false;
    isGpioButtonPressed = false;
    isOnMagnetStrip = false;
    stop_marker_counter = INIT;
    previous_plc_pulse_state = INIT;
  }
}

void DriveController::subtaskInfoCallback(const communication_msgs::RouteInfo& msg)
{
  // Loads current route/task info to enable magnet stop count read from csv file
  stop_marker_counter = INIT;
  isMagnetStopPoint = false;
  isGpioButtonPressed = false;
  previous_plc_pulse_state = INIT;
  if (stop_marker_lookup.find(msg.route) != stop_marker_lookup.end()) {
    current_route = msg.route;
    ROS_INFO("current route is: %s", current_route.c_str());
  } else {
    ROS_INFO("Requested route: %s not found in %s. Setting current route to default", current_route.c_str(), MAGNET_MARKER_LOOKUP_PATH);
    current_route = DEFAULT;
  }
}

void DriveController::inputTwistCallback(const geometry_msgs::TwistStamped& msg)
{
  // twist command from direct_stop node
  direct_stop_twist_cmd = msg;
}

void DriveController::gpioCallback(const std_msgs::Int16& msg)
{
  // Resumes driving after magnet stop when receive signal from Freedom
  if (msg.data == ON && isMagnetStopPoint) {
    isMagnetStopPoint = false;
    isGpioButtonPressed = true;
  }
}

void DriveController::run()
{
  ros::Rate loop_rate(READ_CSV_REFRESH_RATE);
  while (ros::ok()) {
    readCSV();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_controller");
  DriveController mode_controller;
  mode_controller.run();
  return 0;
}
