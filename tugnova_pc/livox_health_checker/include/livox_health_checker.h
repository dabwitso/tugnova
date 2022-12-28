#ifndef LIVOX_HEALTH_CHECKER
#define LIVOX_HEALTH_CHECKER

#include<ros/ros.h>
#include<ros/this_node.h>
#include<ros/master.h>
#include<std_msgs/Int16.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Bool.h>
#include<vector>
#include<array>
#include<string>
#include<stdio.h>
#include<climits>


class HealthChecker{
  public:
    HealthChecker();
    void run();
  private:
    ros::NodeHandle nh;

    ros::Publisher livox_health_pub;
    ros::Publisher livox_error0_pub;
    ros::Publisher livox_error1_pub;
    ros::Publisher livox_error2_pub;

    ros::Subscriber livox_ping_sub;
    ros::Subscriber script_PID_sub;

    // variables
    std::vector<std::string> node_list;

    int script_pid;
    int counter;
    int error_display_count;

    static const int ON = 1;
    static const int INIT = 1;
    static const int NODE_ERROR = 1;
    static const int PING_ERROR = 2;
    static const int CLOCK_RATE = 1; // 1 Hz
    static const int NUMBER_OF_NODES = 3; // 1 Hz

    bool isNodesAlive;
    bool isPingError;

    std::string script_mode;

    const std::string LIVOX_NODE_IP = "192.168.0.40";
    const std::string LIVOX_SCRIPT_PATH = "/home/nvidia/Autoware/ros/nodeshell/spawn_livox_script.sh";
    std::array<std::string,NUMBER_OF_NODES> NODES_TO_TRACK;

    // methods
    void livoxHealthChecker();
    void nodeAliveChecker();

    // callbacks
    void pingCallback(const std_msgs::Int16& msg);
    void pidCallback(const std_msgs::Int32& msg);
};

#endif
