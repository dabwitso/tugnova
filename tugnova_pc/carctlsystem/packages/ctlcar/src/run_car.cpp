#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "run_car");
  ros::NodeHandle n;
  ros::Publisher ctrl_car_pub = n.advertise<std_msgs::String>("state_cmd", 1000);
  ros::Rate loop_rate(10);

  int count = 0;

  while(ros::ok()){
   std_msgs::String msg;
  std::stringstream ss;
  ss << "clear";
  msg.data = ss.str();
  ctrl_car_pub.publish(msg);
  loop_rate.sleep();
  count++;
  if (count >= 30){
     break;

  }  

  }
  
  
  return 0;
  
}
