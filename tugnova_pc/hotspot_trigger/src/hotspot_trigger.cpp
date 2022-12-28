#include "hotspot_trigger.h"

Hotspot::Hotspot()
    : private_nh("~")
{
  // ros handles
  // publisher
  hotspot_error_pub = nh.advertise<std_msgs::Int16>("hotspot_error", 10);
  // subscriber
  freedom_hotspot_sub = nh.subscribe("hotspot_trigger", 1, &Hotspot::hotspotTriggerCallback, this);
  blue_line_sub = nh.subscribe("send_error_status", 1, &Hotspot::isBluelineCallback, this);
  hotspot_script_sub = nh.subscribe("hotspot_error", 1, &Hotspot::hotspotScriptCallback, this);

  // variables
  isHotspotSet = false;
  isSkipBluelineCheck = false;
  hotspot_file = DEFAULT_HOTSPOT;
  BLUE_LINE_NG = { "lost_localization_1_0", "lost_localization_1_2" };
  IGNORE_LIST = { "server_0_0", "server_0_1", "server_2_0", "server_0_2", "server_2_1"};
}

void Hotspot::hotspotScriptCallback(const std_msgs::Int16& msg)
{
  if (msg.data == MISSING_FILE_ERROR_FLG) {
    isSkipBluelineCheck = true;
  }
}

void Hotspot::isBluelineCallback(const std_msgs::String& msg)
{
  if (isHotspotSet) {
    if (std::find(IGNORE_LIST.begin(), IGNORE_LIST.end(), msg.data)
       == IGNORE_LIST.end()){
      std_msgs::Int16 error;
      if (std::find(BLUE_LINE_NG.begin(), BLUE_LINE_NG.end(), msg.data)
          != BLUE_LINE_NG.end()) {
        ROS_INFO("Hotspot Error: BLue line not detected");
        error.data = ON;
      } else {
        ROS_INFO("BLue line detected");
        error.data = OFF;
      }
      hotspot_error_pub.publish(error);
      isHotspotSet = false;
    }
  }
}

void Hotspot::hotspotTriggerCallback(const std_msgs::String& msg)
{
  ROS_INFO("Loading hotspot: %s", msg.data.c_str());
  try {
    hotspot_file = msg.data;
    system(("bash " + SPAWN_HOTSPOT_SCRIPT + " " + hotspot_file).c_str());
    ros::Duration(5.0).sleep();
    if (!isSkipBluelineCheck) {
      isHotspotSet = true;
    }else{
      //reset for next trigger
      isSkipBluelineCheck = false;
    }
  } catch (...) {
    ROS_ERROR("Failed to load hotspot. Check if hotspot directory in renkei/ is set correctly");
  }
}

void Hotspot::run()
{
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hotspot_trigger");
  Hotspot trigger;
  trigger.run();

  return 0;
}
