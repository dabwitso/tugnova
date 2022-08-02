#ifndef PLC_CONNECTOR_H_
#define PLC_CONNECTOR_H_

#include <udp_connector.h>


#pragma pack(1)
#include <udp_msgs/UdpControlPacket.h>
#include <udp_msgs/UdpSensorPacket.h>
#include <udp_msgs/UdpSensorPacket_GreenComs.h>
#pragma pack()

namespace Udp_ns{

enum ComsMode{
    GREEN = 0,
    YUKURI = 1,
};

class PlcSender : public UdpSender{
public:
    PlcSender(std::string _destination, unsigned short _port);
    ~PlcSender();
    void run();
    
private:
    //Node handles
    ros::NodeHandle nh;

    
    //-------------------To send------------------------//
    //UdpControlPacket
    ros::Subscriber udp_control_packet_sub;
    udp_msgs::UdpControlPacket udp_control_packet_msg;
    void udpControlPacketCallback(const udp_msgs::UdpControlPacket& msg);
    void udpControlPacketSend(const udp_msgs::UdpControlPacket& msg);
};

class PlcReceiver : public UdpReceiver{
public:
    PlcReceiver(unsigned short _port);
    ~PlcReceiver();
    void run();
    
private:
    //Node handles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    //Coms
    Udp_ns::ComsMode coms;
    std::string coms_str;
    
    
    //-------------------To receive----------------------//    
    //UdpSensorPacket
    ros::Publisher udp_sensor_packet_pub;
    udp_msgs::UdpSensorPacket udp_sensor_packet_msg;
    udp_msgs::UdpSensorPacket_GreenComs udp_sensor_packet_green_coms_msg;
    void udpSensorPacketReceive();
};

};

#endif //PLC_CONNECTOR_H_
