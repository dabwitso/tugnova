#include <plc_connector.h>

namespace Udp_ns{
    
PlcReceiver::PlcReceiver(unsigned short _port):private_nh("~"),UdpReceiver(_port){
    //Setup coms
    private_nh.param<std::string>("coms", coms_str, "green");
    
    if(coms_str=="green")
	coms = GREEN;
    else if(coms_str=="yukuri")
	coms = YUKURI;
    else
	ROS_ERROR("COMS type has been set wrongly.");

   //Setting for ROS
    switch(coms){
	case GREEN:
	    udp_sensor_packet_pub = nh.advertise<udp_msgs::UdpSensorPacket_GreenComs>("/plc_sensor_packet", 1);
	    break;
	case YUKURI:
    	    udp_sensor_packet_pub = nh.advertise<udp_msgs::UdpSensorPacket>("/plc_sensor_packet", 1);
	    break;
    }
};
        
PlcReceiver::~PlcReceiver(){
    close(socket_var);
};
    
void PlcReceiver::udpSensorPacketReceive(){
    switch(coms){
    case GREEN:
	recv(socket_var, &udp_sensor_packet_green_coms_msg, sizeof(udp_sensor_packet_green_coms_msg), 0); 
	break;
    case YUKURI:
	recv(socket_var, &udp_sensor_packet_msg, sizeof(udp_sensor_packet_msg), 0); 
	break;
    }
};    
    
void PlcReceiver::run(){
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        
        udpSensorPacketReceive();
                
        switch(coms){
	case GREEN:
	    udp_sensor_packet_pub.publish(udp_sensor_packet_green_coms_msg);
	    break;
	case YUKURI:
	    udp_sensor_packet_pub.publish(udp_sensor_packet_msg);
            break;
	}
        loop_rate.sleep();
    }
};
  
};
