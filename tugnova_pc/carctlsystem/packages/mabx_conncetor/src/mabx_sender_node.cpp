#include <mabx_connector.h>

/**
 * MABXへ車両情報を送信.
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "mabx_sender");

    std::string destination;
    int port;

    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("destination", destination, "172.16.1.1");
    private_nh.param<int>("port_send", port, 6001);

    Mabx::MabxSender mabx_sender(destination, port);
    mabx_sender.run();

    return 0;
}
