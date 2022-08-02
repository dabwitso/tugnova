#include <comsctrl_connector.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "comsctrl_sender");
    
    std::string destination;
    int port;
    std::string connection_type_str;
    int connection_type;

    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("destination", destination, "192.168.13.52");
    private_nh.param<int>("port_send", port, 54650);
    private_nh.param<std::string>("connection_type", connection_type_str, "rmpc");
    
    if(connection_type_str=="rmpc"){
        connection_type = 0;
        ROS_INFO("Connection type for the receiver has been set as rmpc.");
    }
    else if(connection_type_str=="localization"){
        connection_type = 1;
        ROS_INFO("Connection type for the receiver has been set as localization.");
    }
    else{
        connection_type = -1;
        ROS_ERROR("Connection type for the receiver has been set with wrong type.");
    }

    Udp_ns::ComsctrlSender comsctrl_sender(destination, port, connection_type);
    comsctrl_sender.run();    
    
    return 0;
}