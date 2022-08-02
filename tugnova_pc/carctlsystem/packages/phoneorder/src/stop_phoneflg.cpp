#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stop_phoneflg");
  ros::NodeHandle n;
  ros::Publisher ctrl_car_pub = n.advertise<std_msgs::Int16>("PhoneOrderFlg", 1000);
  ros::Rate loop_rate(10);

  std_msgs::Int16 msg;
  msg.data = 0; //run:1,stop:0
  int count = 0;

  while(ros::ok()){
    ctrl_car_pub.publish(msg);
    loop_rate.sleep();
    count++;
    if (count >= 30){
      break;
    }  
  }
  
  return 0;
  
}
