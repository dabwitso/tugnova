#include "csv_refresh_clock.h"

CSV_CLOCK::CSV_CLOCK(): private_nh("~"){
  refresh_csv_pub = nh.advertise<std_msgs::Int8>("csv_refresh",10);
}

void CSV_CLOCK::run(){
  ros::Rate loop_rate(READ_CSV_REFRESH_RATE);
  std_msgs::Int8 msg;
  msg.data = 1;
  while(ros::ok()){
    refresh_csv_pub.publish(msg);
    loop_rate.sleep();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "csv_refresh_clock");
  CSV_CLOCK read_clock;
  read_clock.run();
  return 0;
}
