#include "ros/ros.h"
#include "float.h"
#include "autoware_msgs/NDTStat.h"
#include <sstream>

float score = FLT_MAX;
int count = 0;

void ndtStatCallback(const autoware_msgs::NDTStat &ndt_stat)
{
  score = ndt_stat.score < score ? ndt_stat.score : score;

  if (count++ > 10)
  {
    std::cout << score << std::endl; // this standard output is used batch programs.
    exit(0);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sub_ndt_stat");
  ros::NodeHandle n;
  ros::Subscriber sub_ndt_stat = n.subscribe("ndt_stat", 10, ndtStatCallback);

  ros::spin();

  return 0;
}
