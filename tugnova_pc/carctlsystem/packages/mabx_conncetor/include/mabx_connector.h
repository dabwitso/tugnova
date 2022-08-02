#ifndef MABX_CONNECTOR_H
#define MABX_CONNECTOR_H

#include <udp_connector.h>
#include <thread>
#include <chrono>
#include <map>

#pragma pack(1)
#include <udp_msgs/UdpControlPacket.h>
#include <udp_msgs/UdpSensorPacket.h>
#include <carctl_msgs/MabxSenderPacket.h>
#include <carctl_msgs/MabxReceiverPacket.h>
#include <carctl_msgs/monitor_status.h>
#pragma pack()

namespace Mabx {
    class MabxSender : public Udp_ns::UdpSender {
        public:
            MabxSender(std::string _destination, unsigned short _port);
            ~MabxSender();
            void run();
        private:
            ros::NodeHandle nh;
            ros::Subscriber udp_control_packet_sub;
            ros::Subscriber udp_sensor_packet_sub;
            udp_msgs::UdpControlPacket udp_control_packet_msg;
            udp_msgs::UdpSensorPacket udp_sensor_packet_msg;
            
            void udpControlPacketCallback(const udp_msgs::UdpControlPacket& msg);
            void udpSensorPacketCallback(const udp_msgs::UdpSensorPacket& msg);
            carctl_msgs::MabxSenderPacket createMsg(const udp_msgs::UdpControlPacket& control_msg, const udp_msgs::UdpSensorPacket& sensor_msg);
            void udpPacketSend(const carctl_msgs::MabxSenderPacket msg);
    };

    class MabxReceiver : public Udp_ns::UdpReceiver {
        public:
            MabxReceiver(unsigned short _port);
            ~MabxReceiver();
            void run();
        private:
            ros::NodeHandle nh;
            ros::Publisher monitor_status_pub;
            carctl_msgs::MabxReceiverPacket mabx_receiver_packet;
            std::map<uint16_t, carctl_msgs::MabxReceiverPacket> current;
            std::map<uint16_t, carctl_msgs::MabxReceiverPacket> previos;
            std::thread watchdog_timer_thread;
            bool is_watchdog_alive;

            void udpPacketReceive();
            void watchdog_timer();
    };

    static const int16_t INFO = 0;
    static const int16_t ERROR = 1;
};

#endif /* MABX_CONNECTOR_H */