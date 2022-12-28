#include "ndt_stats_health.h"

NDTHealthChecker::NDTHealthChecker()
{
  ndt_stat_error_pub = nh.advertise<carctl_msgs::monitor_status>("/monitor_status", 1);

  ndt_stat_sub = nh.subscribe("/ndt_stat", 1, &NDTHealthChecker::ndtStatCallback, this);
  reset_sub = nh.subscribe("/plc_sensor_packet", 1, &NDTHealthChecker::resetCallback, this);

  last_ndt_signal_time = INITD;
  previous_drive_mode = INIT;
}

void NDTHealthChecker::ndtStatCallback(const autoware_msgs::NDTStat& msg)
{
  ros::Time callback_time = ros::Time::now();
  last_ndt_signal_time = callback_time.toSec();
}

void NDTHealthChecker::resetCallback(const udp_msgs::UdpSensorPacket& msg)
{
  // reset everytime switch mode between auto and manual drive using
  // physical selector switch
  if (msg.ECUMode != previous_drive_mode) {
    ROS_INFO("resetting parameters...");
    // reset time
    ros::Time callback_time = ros::Time::now();
    last_ndt_signal_time = callback_time.toSec();

    // publish clear error status
    carctl_msgs::monitor_status clear_error_msg;
    clear_error_msg.service_name = SERVICE_NAME;
    clear_error_msg.status = ERROR_RESET;
    clear_error_msg.error_msg = "";
    ndt_stat_error_pub.publish(clear_error_msg);
    previous_drive_mode = msg.ECUMode;
  }
}

void NDTHealthChecker::run()
{
  ros::Rate loop_rate(PUBLISH_RATE);
  while (ros::ok()) {
    ros::Time loop_time = ros::Time::now();
    double current_time = loop_time.toSec();
    double time_elapsed = current_time - last_ndt_signal_time;
    if (time_elapsed >= HEART_BEAT_THRESHOLD) {
      ROS_ERROR("Error: heart_beat_threshold exceeded. Time elapsed from last = %d", time_elapsed);
      carctl_msgs::monitor_status ndt_state_msg;
      ndt_state_msg.service_name = SERVICE_NAME;
      ndt_state_msg.status = ERROR;
      ndt_state_msg.error_msg = ERROR_MSG;
      ndt_stat_error_pub.publish(ndt_state_msg);
    } else {
      ROS_INFO("ndt_stat received within specified threshold");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_stat_health");
  NDTHealthChecker health_checker;
  health_checker.run();

  return 0;
}
