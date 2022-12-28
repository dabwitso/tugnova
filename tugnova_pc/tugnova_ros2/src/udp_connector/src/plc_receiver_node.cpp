#include "plc_connector.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int port = 5001;

    PlcReceiver plc_receiver(port);
    plc_receiver.run();

    return 0;
}
