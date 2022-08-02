#include <plc_connector.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "plc_receiver");

    int port;

    ros::NodeHandle private_nh("~");
    
    private_nh.param<int>("port_receive", port, 54650);

    Udp_ns::PlcReceiver plc_receiver(port);
    plc_receiver.run();    
    
    return 0;
}
