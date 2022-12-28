#include "plc_sender.hpp"

using std::placeholders::_1;

PlcSender::PlcSender()
    : Node("plc_sender")
{
    // Setting for ROS
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    udp_control_packet_sub = create_subscription<udp_msgs::msg::UdpControlPacket>("plc_control_packet", qos, std::bind(&PlcSender::udpControlPacketCallback, this, _1));

    // initialize socket
    init();

    // execute run function at rate of 100 Hz (i.e, 10ms)
    // equivalent to while(ros::ok()) in ros1
    timer_ = create_wall_timer(10ms, std::bind(&PlcSender::run, this));

    // variables
    // destination -> plc's IP address
    destination = "192.168.0.10";
}

PlcSender::~PlcSender()
{
    close(socket_var);
}

void PlcSender::init()
{
    // Setting for UDP
    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.sin_addr.s_addr = inet_addr(destination.c_str());
    sock_addr.sin_port = htons(PORT);
    sock_addr.sin_family = AF_INET;

    socket_var = socket(AF_INET, SOCK_DGRAM, 0);
    if (setsockopt(socket_var, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval)))
        RCLCPP_ERROR(get_logger(), "Failed to open socket!");
    else
        RCLCPP_INFO(get_logger(), "Socket open successful!");
}

void PlcSender::udpControlPacketCallback(const udp_msgs::msg::UdpControlPacket& msg)
{
    udp_control_packet_msg = msg;
}

void PlcSender::udpControlPacketSend(const udp_msgs::msg::UdpControlPacket& msg)
{
    int packet_length = sizeof(msg);
    if (sendto(socket_var, &msg, packet_length,
            0, (const sockaddr*)&sock_addr, sizeof(sock_addr))
        < 0) {
        RCLCPP_ERROR(get_logger(), "cannot send command to plc.");
    }
}

void PlcSender::run()
{
    udpControlPacketSend(udp_control_packet_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlcSender>());
    rclcpp::shutdown();
    return 0;
}
