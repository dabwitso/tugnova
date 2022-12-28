#ifndef LAMP_CHECKER_H_
#define LAMP_CHECKER_H_

#include <ros/ros.h>
#include <communication_msgs/LampState.h>
#include <autoware_msgs/Lane.h>
#include <stdint.h>
#include <std_msgs/Int16.h>
#include <cmath>

class LampState{
  public:
    LampState();
    void run();
  private:
    ros::NodeHandle nh;

    ros::Publisher lamp_state_pub;
    ros::Publisher drive_stop_pub;
    ros::Publisher drive_start_pub;

    ros::Subscriber hmi_lamp_sub;
    ros::Subscriber lamp_flag_sub;
    ros::Subscriber lamp_stop_sub;

    // variables
    static const int16_t ON = 1;
    static const int16_t OFF = 0;
    static const int16_t NOW = 0;
    static const int SPECIAL_STOP_FLAG = 100;
    static const int SPECIAL_LAMP_ID = 4;
    static const int ID_NORMALIZER_FACTOR = 10;

    int publish_lock;

    bool isSpecialStop;

    // callbacks
    void hmiLampCallback(const std_msgs::Int16& msg);
    void lampWaypointCallback(const autoware_msgs::Lane& msg);
    void lampStopCallback(const autoware_msgs::Lane& msg);

};

#endif
