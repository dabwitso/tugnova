#include <mabx_connector.h>

/**
 * MABXから状態を受信.
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "mabx_receiver");

    std::string destination;
    int port;

    ros::NodeHandle private_nh("~");

    private_nh.param<int>("port_receive", port, 6000);

    Mabx::MabxReceiver mabx_receiver(port);
    mabx_receiver.run();

    return 0;
}
