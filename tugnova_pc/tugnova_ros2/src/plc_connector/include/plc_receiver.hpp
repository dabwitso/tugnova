#ifndef PLC_RECEIVER_H_
#define PLC_RECEIVER_H_

#include "rclcpp/rclcpp.hpp"
#include "udp_msgs/msg/udp_sensor_packet.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <memory>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

using namespace std::chrono_literals;

class PlcReceiver : public rclcpp::Node {
  public:
  PlcReceiver();
  ~PlcReceiver();
  void run();

  private:
  // ros handles
  rclcpp::Publisher<udp_msgs::msg::UdpSensorPacket>::SharedPtr udp_sensor_packet_pub;
  udp_msgs::msg::UdpSensorPacket udp_sensor_packet_msg;

  rclcpp::TimerBase::SharedPtr timer_;

  // vairables
  int socket_var;

  static const int PORT = 5001;

  struct sockaddr_in sock_addr;
  struct timeval tv;

  // methods
  void init();
  void udpSensorPacketReceive();
};
#endif
