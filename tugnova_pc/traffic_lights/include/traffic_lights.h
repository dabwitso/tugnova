#ifndef TRAFFIC_LIGHTS_H
#define TRAFFIC_LIGHTS_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <stdint.h>
#include <autoware_msgs/Lane.h>
#include <array>
#include <algorithm>

class Traffic_Lights{
  public:
    Traffic_Lights();
    void run();
  private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Publisher traffic_light_flg_pub;
    ros::Publisher traffic_stop_flg_pub;

    ros::Subscriber traffic_flg_sub;

    // variables
    int traffic_light_flg;
    int traffic_flg_publish_count;

    static const int16_t ON = 1;
    static const int16_t OFF = 0;
    static const int16_t INIT = 0;
    static const int16_t NOW = 0;


    // TRAFFIC_LIGHT_STOP_FLAG are the first value of a pair of flags assigned to a traffic light area
    // example traffic light zone pairs: (1,2),(3,4),(5,6)
    // initialized in .cpp file. adjust the values accordingly when needed
    // e.g TRAFFIC_LIGHT_STOP_FLAG = {1, 3, 5} according to given example pairs
    std::array<int, 3> TRAFFIC_LIGHT_STOP_FLAG;

    // callbacks
    void trafficLightCallback(const autoware_msgs::Lane& msg);


};

#endif
