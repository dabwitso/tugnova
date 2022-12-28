#ifndef STATUS_CODE_H
#define STATUS_CODE_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <array>
#include "csv.h"

class AutowareState{
  public:
    AutowareState();
    void run();
  private:
    // ros setup
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Publisher vehicle_state_pub;
    ros::Publisher error_code_pub;

    ros::Subscriber autoware_health_sub;
    ros::Subscriber vehicle_operation_sub;

    // variables
    const std::string STATUS_CODE_PATH = "/home/nvidia/Autoware/ros/renkei/DICTIONARY/STATUS_CODES.csv";
    const std::string IDLE = "status_0_0";
    const std::string RUNNING = "status_0_1";
    const std::string BOOT_ERROR = "status_1_0";

    std::unordered_map<std::string, int> status_code_map;
    std::array<std::string, 3> VEHICLE_STATES;

    // misc functions
    void read_csv();

    // callbacks
    void healthCallback(const std_msgs::String& msg);
    void vehicleStateCallback(const std_msgs::String& msg);
};

#endif


