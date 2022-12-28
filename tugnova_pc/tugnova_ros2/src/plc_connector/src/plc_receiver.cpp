#include "plc_receiver.hpp"

PlcReceiver::PlcReceiver()
    : Node("plc_receiver")
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  udp_sensor_packet_pub = create_publisher<udp_msgs::msg::UdpSensorPacket>("plc_sensor_packet", qos);
  // publication rate timer initialization. 10ms -> 100 Hz
  timer_ = create_wall_timer(10ms, std::bind(&PlcReceiver::run, this));

  // variable initialization
  tv.tv_sec = 0.01;
  tv.tv_usec = 0;

  // initialize socket
  init();

}

PlcReceiver::~PlcReceiver()
{
  close(socket_var);
}

void PlcReceiver::init()
{
    // Setting for UDP
    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.sin_port = htons(PORT);
    sock_addr.sin_family = AF_INET;
    sock_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    socket_var = socket(AF_INET, SOCK_DGRAM, 0);
    if (setsockopt(socket_var, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval)))
        perror("Error in setsockopt()");
    perror("Failed to initialize socket");
}

void PlcReceiver::udpSensorPacketReceive()
{
    recv(socket_var, &udp_sensor_packet_msg, sizeof(udp_sensor_packet_msg), 0);
}

void PlcReceiver::run()
{
  auto msg = udp_msgs::msg::UdpSensorPacket();
  udpSensorPacketReceive();
  udp_sensor_packet_pub->publish(msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlcReceiver>());
  rclcpp::shutdown();
  return 0;
}
