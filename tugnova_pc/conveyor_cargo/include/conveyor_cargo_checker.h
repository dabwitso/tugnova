#ifndef CONVEYOR_CARGO_CHECK_H
#define CONVEYOR_CARGO_CHECK_H

#include <ros/ros.h>
#include <communication_msgs/PlcConveyor.h>
#include <autoware_msgs/Lane.h>
#include <udp_msgs/UdpSensorPacket.h>

class ConveyorCheck{
  public:
    ConveyorCheck();
    void run();
  private:
    ros::NodeHandle nh;

    ros::Publisher conveyor_state_pub;

    ros::Subscriber plc_sensor_sub;
    ros::Subscriber magnet_drive_sub;

    bool isMagnetDriveMode;

    static const int NOW = 0;
    static const int MAGNET_DRIVE_MODE = 2;

    // callback
    void plcSensorPacketCallback(const udp_msgs::UdpSensorPacket& msg);
    void magnetDriveCallback(const autoware_msgs::Lane& msg);
};

#endif
