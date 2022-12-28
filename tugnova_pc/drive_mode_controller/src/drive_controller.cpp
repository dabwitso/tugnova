#include "drive_controller.h"

DriveController::DriveController()
    : private_nh("~")
{
  // ros handles
  // publication
  // twist_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("out_twist_ctrl", 10);
  waitpoint_pub = nh.advertise<std_msgs::Bool>("waitpoint_state", 10);
  magnet_drive_state_pub = nh.advertise<std_msgs::String>("magnet_status", 1);
  magnet_stop_plc_pub = nh.advertise<std_msgs::Int16>("magnet_stop", 10);
  stop_drive_pub = nh.advertise<std_msgs::Int16>("stop_drive", 10);

  // subscription
  plc_sensor_sub = nh.subscribe("plc_sensor_packet", 10, &DriveController::plcSensorCallback, this);
  drive_mode_sub = nh.subscribe("safety_waypoints", 10, &DriveController::driveModeCallback, this);
  gpiostartflg_sub = nh.subscribe("GpioStartFlg", 10, &DriveController::gpioCallback, this);
  subtask_name_sub = nh.subscribe("route_cmds", 10, &DriveController::subtaskInfoCallback, this);
  reset_flags_sub = nh.subscribe("magnet_drive_reset", 10, &DriveController::resetCallback, this);
  cart_id_sub = nh.subscribe("cart_id", 10, &DriveController::cartidCallback, this);
  csv_refresh_clock_sub = nh.subscribe("csv_refresh", 10, &DriveController::csvRefreshTriggerCallback, this);

  private_nh.param<std::string>("waypoint_job_handler/current_route",current_route,DEFAULT);

  // variable initialization
  isMagnetDrive = false;
  isMagnetStopPoint = false;
  isGpioButtonPressed = false;
  isOnMagnetStrip = false;
  isMagnetDataRecorded = false;
  isStartTimeRecorded = false;
  isCsvRefresh = false;

  stop_marker_counter = INIT;
  previous_plc_pulse_state = INIT;
  cart_stop_time = INIT;
  dict_stop_point = INT_MAX;

  cart_id = DEFAULT_CART_ID;
}

void DriveController::csvRefreshTriggerCallback(const std_msgs::Int8& msg)
{
  if (msg.data == ON) {
    if (isCsvRefresh) {
      // read csv file in intervals set by csv_refresh_clock node
      readCSV();
    }
  }
}

void DriveController::readCSV()
{
  // read magnet_count.csv file
  if (!magnet_count_lookup.empty()) {
    // useful since lookup refreshes from file every refresh cycle
    magnet_count_lookup.clear();
  }
  io::CSVReader<7> in(MAGNET_MARKER_LOOKUP_PATH);
  in.read_header(io::ignore_extra_column, "route_id", "stop_position", "cart1", "cart2", "cart3", "cart4", "cart5");
  std::string route_id;
  size_t cart1, cart2, cart3, cart4, cart5;
  int stop_position;

  while (in.read_row(route_id, stop_position, cart1, cart2, cart3, cart4, cart5)) {
    magnet_count_lookup[route_id][stop_position]["cart1"] = cart1;
    magnet_count_lookup[route_id][stop_position]["cart2"] = cart2;
    magnet_count_lookup[route_id][stop_position]["cart3"] = cart3;
    magnet_count_lookup[route_id][stop_position]["cart4"] = cart4;
    magnet_count_lookup[route_id][stop_position]["cart5"] = cart5;
  }
  ROS_INFO("Read magnet_count.csv");
}

void DriveController::stopPublisher()
{
  // publish stop drive to drive_stop_controller node
  std_msgs::Int16 drive_cmd;
  drive_cmd.data = ON;
  stop_drive_pub.publish(drive_cmd);

  // send signal to plc signifying stopped by magnet
  std_msgs::Int16 magnet_plc_stop_cmd;
  magnet_plc_stop_cmd.data = 1;
  magnet_stop_plc_pub.publish(magnet_plc_stop_cmd);
}

void DriveController::cartidCallback(const std_msgs::String& msg)
{
  cart_id = msg.data;
}

void DriveController::plcSensorCallback(const udp_msgs::UdpSensorPacket& msg)
{

  if (msg.ECUMode == MANUAL_DRIVE) {
    isMagnetStopPoint = false;
    // reset magnet mode plc if selector turned to manual
    // ROS_INFO("Manual drive mode detected");
    std_msgs::Int16 magnet_plc_stop_cmd;
    magnet_plc_stop_cmd.data = 0;
    magnet_stop_plc_pub.publish(magnet_plc_stop_cmd);
  }

  if (isMagnetDrive) {
    // Magnet drive mode
    if (isMagnetStopPoint) {
      stopPublisher();
    } else {
      int current_stop_pt = msg.AccelPotVol;
      // ROS_WARN("plc flag: %d", current_stop_pt);
      if (current_stop_pt != previous_plc_pulse_state && current_stop_pt == 1) {
        ++stop_marker_counter;
        previous_plc_pulse_state = current_stop_pt;
      } else
        previous_plc_pulse_state = current_stop_pt;

      // check if stop point reached

      if (current_route != DEFAULT) {
        if (!isMagnetDataRecorded) {
          isCsvRefresh = false;
          dict_stop_point = magnet_count_lookup[current_route].begin()->first;
          cart_stop_time = magnet_count_lookup[current_route][dict_stop_point][cart_id];
          isMagnetDataRecorded = true;
          isCsvRefresh = true;
        }
      }
      // ROS_INFO("target count: %d    current count: %d", dict_stop_point, stop_marker_counter);
      if (stop_marker_counter == dict_stop_point) {
        // stop point reached.
        if (!isGpioButtonPressed) {
          if (!isStartTimeRecorded) {
            start_time = chrono_clock::now();
            isStartTimeRecorded = true;
          }
          auto current_time = chrono_clock::now();
          size_t time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
          // ROS_INFO("cart_time: %i   ->   time_elapsed: %i ",(int)cart_stop_time, (int)time_elapsed);

          // cart stop time elapsed to effect zero velocity
          ROS_INFO("cart_time: %d   ->   time_elapsed: %d ", (int)cart_stop_time, (int)time_elapsed);

          if (time_elapsed >= cart_stop_time && time_elapsed <= (cart_stop_time + 25)) {
            ROS_INFO("Magnet stop point reached. Waiting for waitpoint clear command");
            isMagnetStopPoint = true;
            std_msgs::String message;
            message.data = "stop";
            magnet_drive_state_pub.publish(message);
          }
        }
      }
    }
  } else {
    // Autoware drive mode
    isMagnetStopPoint = false;
    stop_marker_counter = INIT;
    previous_plc_pulse_state = INIT;
    isGpioButtonPressed = false;
    isStartTimeRecorded = false;
    isMagnetDataRecorded = false;
    dict_stop_point = INT_MAX;
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
    isMagnetDataRecorded = false;
    isStartTimeRecorded = false;
    stop_marker_counter = INIT;
    previous_plc_pulse_state = INIT;
    dict_stop_point = INT_MAX;
  }
}

void DriveController::subtaskInfoCallback(const communication_msgs::RouteInfo& msg)
{
  dict_stop_point = INT_MAX;
  isCsvRefresh = false;
  // Loads current route/task info to enable magnet stop count read from csv file
  stop_marker_counter = INIT;
  isMagnetStopPoint = false;
  isGpioButtonPressed = false;
  isStartTimeRecorded = false;
  isMagnetDataRecorded = false;
  previous_plc_pulse_state = INIT;
  if (magnet_count_lookup.find(msg.route) != magnet_count_lookup.end()) {
    current_route = msg.route;
    ROS_INFO("current route is: %s", current_route.c_str());
  } else if (msg.route == "finish") {
    ROS_INFO("End of csv file reached");
  } else {
    current_route = msg.route;
    ROS_ERROR("Requested route: %s not found in %s. Setting current route to default", current_route.c_str(), MAGNET_MARKER_LOOKUP_PATH.c_str());
    current_route = DEFAULT;
    ROS_INFO("Requested route: %s not found in Autoware/ROUTE/magnet_count.csv. Setting current route to default", current_route.c_str());
    current_route = "default";
  }
  isCsvRefresh = true;
}

void DriveController::gpioCallback(const std_msgs::Int16& msg)
{
  // Resumes driving after magnet stop when receive signal from Freedom
  if (msg.data == ON && isMagnetStopPoint) {
    isMagnetStopPoint = false;
    isGpioButtonPressed = true;
    isStartTimeRecorded = false;
    isMagnetDataRecorded = false;
    // send signal to plc signifying stopped by magnet
    std_msgs::Int16 magnet_plc_stop_cmd;
    magnet_plc_stop_cmd.data = 0;
    magnet_stop_plc_pub.publish(magnet_plc_stop_cmd);
  }
}

void DriveController::run()
{
  readCSV();
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_controller");
  DriveController mode_controller;
  mode_controller.run();
  return 0;
}
