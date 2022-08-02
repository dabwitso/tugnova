#include <plc_connector.h>

namespace Udp_ns{
    
PlcSender::PlcSender(std::string _destination, unsigned short _port):UdpSender(_destination, _port){
    //Setting for ROS
    udp_control_packet_sub = nh.subscribe("/plc_control_packet", 1, &Udp_ns::PlcSender::udpControlPacketCallback, this);
};
    
PlcSender::~PlcSender(){
    close(socket_var);
};
    
void PlcSender::udpControlPacketCallback(const udp_msgs::UdpControlPacket& msg){
    udp_control_packet_msg = msg;
};
    
void PlcSender::udpControlPacketSend(const udp_msgs::UdpControlPacket& msg){
    int packet_length = sizeof(msg);
    if( sendto(socket_var, &msg, packet_length, 0, (const sockaddr*)&sock_addr, sizeof(sock_addr)) < 0 )
    {
      ROS_ERROR("cannot send command" );
    }
};
    
void PlcSender::run(){
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        
        udpControlPacketSend(udp_control_packet_msg);
        
        loop_rate.sleep();
    }
};

};
