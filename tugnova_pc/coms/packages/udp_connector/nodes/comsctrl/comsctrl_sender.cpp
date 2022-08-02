#include <comsctrl_connector.h>

namespace Udp_ns{
    
ComsctrlSender::ComsctrlSender(std::string _destination, unsigned short _port, int _connection_type):UdpSender(_destination, _port){
    connection_type = _connection_type;

    //Setting for ROS
    switch(connection_type){
    case 0:
        control_info_sub = nh.subscribe("/comsctrl_control_info", 1, &Udp_ns::ComsctrlSender::controlInfoCallback, this);
        break;
    case 1:
        localization_info_sub = nh.subscribe("/comsctrl_localization_info", 1, &Udp_ns::ComsctrlSender::localizationInfoCallback, this);
        break;
    default:
        ROS_ERROR("Connection type error: check the input type number into the constructor.");
        break;
    }
};
    
ComsctrlSender::~ComsctrlSender(){
    close(socket_var);
};
    
void ComsctrlSender::controlInfoCallback(const udp_msgs::ControlInfo& msg){
    control_info_msg = msg;
};
    
void ComsctrlSender::controlInfoSend(const udp_msgs::ControlInfo& msg){
    char buffer[100];
    char to_send_text[1000] = "";
    
    sprintf(buffer,"tire_angle_ref:%f\n",msg.tire_angle);
    strcat(to_send_text,buffer);
    sprintf(buffer,"velocity_ref:%f\n",msg.velocity);
    strcat(to_send_text,buffer);

    sendto(socket_var, to_send_text, strlen(to_send_text)+1, 0, (const sockaddr*)&sock_addr, sizeof(sock_addr));
};
    
void ComsctrlSender::localizationInfoCallback(const udp_msgs::LocalizationInfo& msg){
    localization_info_msg = msg;
};

void ComsctrlSender::localizationInfoSend(const udp_msgs::LocalizationInfo& msg){
    char buffer[70];
    char to_send_text[500] = "";
    
    sprintf(buffer,"x:%f\n",msg.x);
    strcat(to_send_text,buffer);
    sprintf(buffer,"y:%f\n",msg.y);
    strcat(to_send_text,buffer);
    sprintf(buffer,"yaw:%f\n",msg.yaw);
    strcat(to_send_text,buffer);
    sprintf(buffer,"velocity:%f\n",msg.velocity);
    strcat(to_send_text,buffer);
    sprintf(buffer,"yawrate:%f\n",msg.yawrate);
    strcat(to_send_text,buffer);
    sprintf(buffer,"iteration:%d\n",msg.iteration);
    strcat(to_send_text,buffer);
    sprintf(buffer,"score:%f\n",msg.score);
    strcat(to_send_text,buffer);

    sendto(socket_var, to_send_text, strlen(to_send_text)+1, 0, (const sockaddr*)&sock_addr, sizeof(sock_addr));
}
    
    
void ComsctrlSender::run(){
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        
        switch(connection_type){
            //RMPC
            case 0:
                controlInfoSend(control_info_msg);
                break;
            //Localization
            case 1:
                localizationInfoSend(localization_info_msg);
		        break;
            default:
                ROS_ERROR("Cannot send the msg because the connection type has been set in wrong number");
                break;
        }
        
        loop_rate.sleep();
    }
};

};
