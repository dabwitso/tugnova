#ifndef HOTSPOT_TRIGGER_H
#define HOTSPOT_TRIGGER_H

#include <algorithm>
#include <array>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <string>

class Hotspot {
  public:
  Hotspot();
  void run();

  private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Publisher hotspot_error_pub;

  ros::Subscriber freedom_hotspot_sub;
  ros::Subscriber blue_line_sub;
  ros::Subscriber hotspot_script_sub;

  static const int ON = 1;
  static const int OFF = 0;
  static const int MISSING_FILE_ERROR_FLG = 2;

  bool isHotspotSet;
  bool isSkipBluelineCheck;

  const std::string DEFAULT_HOTSPOT = "23E_A";
  std::array<std::string, 2> BLUE_LINE_NG;
  std::array<std::string, 5> IGNORE_LIST;
  const std::string SPAWN_HOTSPOT_SCRIPT = "/home/nvidia/Autoware/ros/nodeshell/set_hotspot.sh";
  std::string hotspot_file;

  // callbacks
  void hotspotTriggerCallback(const std_msgs::String& msg);
  void isBluelineCallback(const std_msgs::String& msg);
  void hotspotScriptCallback(const std_msgs::Int16& msg);
};

#endif
