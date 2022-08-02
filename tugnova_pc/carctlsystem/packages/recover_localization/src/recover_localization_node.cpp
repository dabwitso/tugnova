#include <recover_localization.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "recover_localization");

    int buffer_size;
    int window_size;
    double threshold;
    std::string mode;

    ros::NodeHandle private_nh("~");

    private_nh.param<int>("buffer_size", buffer_size, 1000);
    private_nh.param<int>("window_size", window_size, 5);
    private_nh.param<double>("threshold", threshold, 1.0);
    private_nh.param<std::string>("mode", mode, "odometry");

    RecoverLocalization::Node recover_localization(buffer_size, window_size, threshold, mode);
    recover_localization.run();

    return 0;
}