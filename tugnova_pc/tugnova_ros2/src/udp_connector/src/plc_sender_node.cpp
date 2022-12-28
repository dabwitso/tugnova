#include "plc_connector.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string destination = "192.168.0.10";
    int port = 5002;

    PlcSender plc_sender(destination, port);
    plc_sender.run();

    return 0;
}
