#include "plc_connector.hpp"

using std::placeholders::_1;


PlcSender::PlcSender(std::string _destination, unsigned short _port) : UdpSender(_destination, _port)
{
    // Setting for ROS
    Node("plc_sender");

    rclcpp::QoS qos(rclcpp::KeepLast(10));

    udp_control_packet_sub = create_subscription<udp_msgs::msg::UdpControlPacket>("plc_control_packet", qos, std::bind(&PlcSender::udpControlPacketCallback, this, _1));
};

PlcSender::~PlcSender()
{
    close(socket_var);
};

void PlcSender::udpControlPacketCallback(const udp_msgs::msg::UdpControlPacket &msg)
{
    udp_control_packet_msg = msg;
};

void PlcSender::udpControlPacketSend(const udp_msgs::msg::UdpControlPacket &msg)
{
    int packet_length = sizeof(msg);
    if (sendto(socket_var, &msg, packet_length, 0, (const sockaddr *)&sock_addr, sizeof(sock_addr)) < 0)
    {
        RCLCPP_INFO(get_logger(), "cannot send command");
    }
};

void PlcSender::run()
{
    rclcpp::Rate loop_rate(100);
    while (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        // executor.add_node(Node);

        executor.spin_once();

        udpControlPacketSend(udp_control_packet_msg);

        loop_rate.sleep();
    }
};

