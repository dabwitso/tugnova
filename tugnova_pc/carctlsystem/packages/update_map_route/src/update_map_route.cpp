#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <sstream>

bool run_flg = false;

void changeRouteCallback(const std_msgs::Int8::ConstPtr& msg)
{
  //printf ("data:%d\n", msg->data);
  if (msg->data == 1)
  {
    if (run_flg == false)
    {
      run_flg = true;
      system("~/Autoware/ros/nodeshell/UpdateMapRoute.sh");
      run_flg = false;
    }
    
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "update_map_route");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("RouteChangeFlg", 1000, changeRouteCallback);
  ros::spin();
  
  return 0;
  
}
