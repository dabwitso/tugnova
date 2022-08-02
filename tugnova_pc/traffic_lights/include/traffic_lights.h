#ifndef TRAFFIC_LIGHTS_H
#define TRAFFIC_LIGHTS_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <stdint.h>
#include <autoware_msgs/Lane.h>

class Traffic_Lights{
  public:
    Traffic_Lights();
    void run();
  private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Publisher traffic_flg_pub;
    ros::Publisher traffic_stop_flg_pub;

    ros::Subscriber traffic_flg_sub;

    // variables
    int traffic_light_flg;
    int traffic_flg_publish_count;

    static const int16_t ON = 1;
    static const int16_t OFF = 0;
    static const int16_t INIT = 0;
    static const int16_t NOW = 0;



    // callbacks
    void trafficLightCallback(const autoware_msgs::Lane& msg);


};

#endif
