#include "plc_connector.hpp"

PlcReceiver::PlcReceiver(unsigned short _port) : UdpReceiver(_port)
{
    // Setup coms
    rclcpp::Node("plc_receiver");

    // Setting for ROS
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    udp_sensor_packet_pub = create_publisher<udp_msgs::msg::UdpSensorPacket>("plc_sensor_packet", qos);
};

PlcReceiver::~PlcReceiver()
{
    close(socket_var);
};

void PlcReceiver::udpSensorPacketReceive()
{

    recv(socket_var, &udp_sensor_packet_msg, sizeof(udp_sensor_packet_msg), 0);

};

void PlcReceiver::run()
{
    rclcpp::Rate loop_rate(100);
    while (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;

        // executor.add_node(Node);

        executor.spin_once();

        udpSensorPacketReceive();

        udp_sensor_packet_pub->publish(udp_sensor_packet_msg);

        loop_rate.sleep();
    }
};

