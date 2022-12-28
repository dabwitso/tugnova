#include "conveyor_cargo_checker.h"

ConveyorCheck::ConveyorCheck()
{
  // ros handles
  conveyor_state_pub = nh.advertise<communication_msgs::PlcConveyor>("/conveyor_cargo_state", 1);

  plc_sensor_sub = nh.subscribe("plc_sensor_packet", 1, &ConveyorCheck::plcSensorPacketCallback, this);
  magnet_drive_sub = nh.subscribe("safety_waypoints", 1, &ConveyorCheck::magnetDriveCallback, this);

  isMagnetDriveMode = false;
}

void ConveyorCheck::plcSensorPacketCallback(const udp_msgs::UdpSensorPacket& msg)
{
  if (isMagnetDriveMode) {
    communication_msgs::PlcConveyor cargo_state;
    cargo_state.stopper_state = msg.Inv_V;
    cargo_state.parts_position = msg.Inv_A;
    cargo_state.cargo_mode = msg.Acc_trq;

    conveyor_state_pub.publish(cargo_state);
  }
}

void ConveyorCheck::magnetDriveCallback(const autoware_msgs::Lane& msg)
{
  if (msg.waypoints[NOW].wpc.mode_change == MAGNET_DRIVE_MODE)
    isMagnetDriveMode = true;
  else
    isMagnetDriveMode = false;
}

void ConveyorCheck::run()
{
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "conveyor_cargo_checker");
  ConveyorCheck checker;
  checker.run();

  return 0;
}
