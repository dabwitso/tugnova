#ifndef PLC_CONNECTOR_H_
#define PLC_CONNECTOR_H_

#include "udp_connector.hpp"
#pragma pack(1)
#include <udp_msgs/msg/udp_control_packet.hpp>
#include <udp_msgs/msg/udp_sensor_packet.hpp>
#pragma pack()


class PlcSender : public UdpSender
{
    public:
        PlcSender(std::string _destination, unsigned short _port);
        ~PlcSender();
        void run();

    private:
        // UdpControlPacket
        rclcpp::Subscription<udp_msgs::msg::UdpControlPacket>::SharedPtr udp_control_packet_sub;
        udp_msgs::msg::UdpControlPacket udp_control_packet_msg;
        void udpControlPacketCallback(const udp_msgs::msg::UdpControlPacket& msg);
        void udpControlPacketSend(const udp_msgs::msg::UdpControlPacket& msg);
};

class PlcReceiver : public UdpReceiver
{
    public:
        PlcReceiver(unsigned short _port);
        ~PlcReceiver();
        void run();

    private:
        //-------------------To receive----------------------//
        // UdpSensorPacket
        rclcpp::Publisher<udp_msgs::msg::UdpSensorPacket>::SharedPtr udp_sensor_packet_pub;
        udp_msgs::msg::UdpSensorPacket udp_sensor_packet_msg;
        void udpSensorPacketReceive();
};


#endif // PLC_CONNECTOR_H_
