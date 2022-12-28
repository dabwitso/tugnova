#include "status_code.h"

AutowareState::AutowareState(): private_nh("~"){
  // ros handles
  // publishers
  vehicle_state_pub = nh.advertise<std_msgs::String>("/vehicle_operation_status",10);
  error_code_pub = nh.advertise<std_msgs::Int16>("/autoware_status_code",10);

  // subscriber
  autoware_health_sub = nh.subscribe("send_error_status",1, &AutowareState::healthCallback, this);
  vehicle_operation_sub = nh.subscribe("send_error_status",1, &AutowareState::vehicleStateCallback, this);

  // variables
  VEHICLE_STATES = {"status_0_0","status_0_1","status_1_0"};
}

void AutowareState::read_csv(){
  try{
    io::CSVReader<2> in(STATUS_CODE_PATH);
    in.read_header(io::ignore_extra_column, "code_id","code_num");
    std::string code_id;
    int code_num;
    ROS_INFO("Reading %s",STATUS_CODE_PATH.c_str());
    while (in.read_row(code_id,code_num)){
      status_code_map[code_id]=code_num;
    }
    ROS_INFO("Done reading error status codes");
  }catch (...){
    ROS_ERROR("File not found : %s. Aborting file read operation...",STATUS_CODE_PATH.c_str());
  }
}

void AutowareState::healthCallback(const std_msgs::String& msg){
  std_msgs::Int16 error;
  error.data = status_code_map.find(msg.data)!=status_code_map.end()?status_code_map[msg.data]:0;
  if (error.data != 0){
    error_code_pub.publish(error);
  }
}

void AutowareState::vehicleStateCallback(const std_msgs::String& msg){
  std_msgs::String vstate;
  if(std::find(VEHICLE_STATES.begin(),VEHICLE_STATES.end(),msg.data) != VEHICLE_STATES.end()){
    std::string state = msg.data;
    if (state == IDLE){
        vstate.data = "idle";
        vehicle_state_pub.publish(vstate);
    }else if (state == RUNNING){
        vstate.data = "running";
        vehicle_state_pub.publish(vstate);
    }else if (state == BOOT_ERROR){
        vstate.data = "boot_error";
        vehicle_state_pub.publish(vstate);
    }else{
        vstate.data = "unknown";
        vehicle_state_pub.publish(vstate);
    }
  }else{
    vstate.data = "manual_mode";
    vehicle_state_pub.publish(vstate);
  }
}

void AutowareState::run(){
  read_csv();
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "autoware_status");
  AutowareState states;
  states.run();

  return 0;
}
