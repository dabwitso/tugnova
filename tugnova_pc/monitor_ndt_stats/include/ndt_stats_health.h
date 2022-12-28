#ifndef NDT_STATS_HEALTH_H
#define NDT_STATS_HEALTH_H

#include<ros/ros.h>
#include<autoware_msgs/NDTStat.h>
#include<carctl_msgs/monitor_status.h>
#include<udp_msgs/UdpSensorPacket.h>
#include<string>

class NDTHealthChecker{
  public:
    NDTHealthChecker();
    void run();
  private:
    ros::NodeHandle nh;

    ros::Publisher ndt_stat_error_pub;
    ros::Subscriber ndt_stat_sub;
    ros::Subscriber reset_sub;

    const double PUBLISH_RATE = 100; // 100Hz
    const double HEART_BEAT_THRESHOLD = 0.3; // in secs
    const double INITD = 0.0;

    double last_ndt_signal_time;

    int previous_drive_mode;

    static const int INIT = 0;
    static const int ERROR = 1;
    static const int ERROR_RESET = 0;

    const std::string SERVICE_NAME = "ndt_stat_health_checker";
    const std::string ERROR_MSG = "ouster NG";

    void ndtStatCallback(const autoware_msgs::NDTStat& msg);
    void resetCallback(const udp_msgs::UdpSensorPacket& msg);
};

#endif
