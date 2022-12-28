#ifndef PLC_SENDER_H_
#define PLC_SENDER_H_

#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <time.h>
#include <udp_msgs/msg/udp_control_packet.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PlcSender : public rclcpp::Node
{
    public:
        PlcSender();
        ~PlcSender();
        void run();

    private:
        // ros handles

        rclcpp::Subscription<udp_msgs::msg::UdpControlPacket>::SharedPtr udp_control_packet_sub;
        udp_msgs::msg::UdpControlPacket udp_control_packet_msg;

        rclcpp::TimerBase::SharedPtr timer_;

        // variables
        int socket_var;
        const int PORT = 5002;

        std::string destination;

        struct sockaddr_in sock_addr;
        struct timeval tv;

        //methods
        void udpControlPacketSend(const udp_msgs::msg::UdpControlPacket& msg);
        void init();

        // callback
        void udpControlPacketCallback(const udp_msgs::msg::UdpControlPacket& msg);
};

#endif
