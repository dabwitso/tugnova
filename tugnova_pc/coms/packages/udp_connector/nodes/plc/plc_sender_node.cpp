#include <plc_connector.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "plc_sender");
    
    std::string destination;
    int port;

    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("destination", destination, "192.168.13.52");
    private_nh.param<int>("port_send", port, 54650);

    Udp_ns::PlcSender plc_sender(destination, port);
    plc_sender.run();    
    
    return 0;
}