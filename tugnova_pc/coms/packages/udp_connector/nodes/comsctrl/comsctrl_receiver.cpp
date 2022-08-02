#include <comsctrl_connector.h>

namespace Udp_ns{
    
ComsctrlReceiver::ComsctrlReceiver(unsigned short _port, int _connection_type):UdpReceiver(_port){
    tv.tv_sec = 0.01;
    tv.tv_usec = 0;

    connection_type = _connection_type;

    //Setting for ROS
    switch(connection_type){
    case 0:
        car_info_pub = nh.advertise<udp_msgs::CarInfo>("/comsctrl_car_info", 1);
        break;
    case 1:
        break;
    default:
        ROS_ERROR("Connection type error: check the input type number into the constructor.");
        break;
    }

};
    
ComsctrlReceiver::~ComsctrlReceiver(){
    close(socket_var);
};
    
void ComsctrlReceiver::carInfoReceive(){
    //int status;
    char buffer[BUFLEN];
    
    memset(buffer, '\0', BUFLEN);
    if(recvfrom(socket_var, buffer, BUFLEN, 0, NULL, NULL) < 0){
        std::cout<<"UDP timeout!"<<std::endl;
        return;
    }
    
    std::string str = "";
    std::string received_text = "";
    received_text.append(buffer);
    std::istringstream stream(received_text);
    while(getline(stream, str)){
        std::string str2 = "";
        std::istringstream stream2(str);
        Udp_ns::CarInfoMode mode;
        int ic = 0;
        
        while(getline(stream2, str2, ':')){
            if(ic == 0){
                if(str2 == "x") mode = X;
                else if(str2 == "y") mode = Y;
                else if(str2 == "velocity") mode = VELOCITY;
                else if(str2 == "yaw") mode = YAW;
                else if(str2 == "tire_angle") mode = TIRE_ANGLE;
	            else if(str2 == "datackr") mode = DATACKR;
                else{
                    std::cout<<"Invalid label!"<<std::endl;
                    break;
                }
                ic++;
            }
            else{
                switch(mode){
                    case X:
                        car_info_msg.x = ::atof(str2.c_str());
			            break;
                    case Y:
                        car_info_msg.y = ::atof(str2.c_str());
			            break;
                    case VELOCITY:
                        car_info_msg.velocity = ::atof(str2.c_str());
			            break;
                    case YAW:
                        car_info_msg.yaw = ::atof(str2.c_str());
			            break;
                    case TIRE_ANGLE:
                        car_info_msg.tire_angle = ::atof(str2.c_str());
			            break;
		            case DATACKR:
                        car_info_msg.datachecker = ::atof(str2.c_str());
			            break;
                    default:
                        std::cout<<"Invalid label!"<<std::endl;
                        break;
                }
            }
        }
    }
};  
    
void ComsctrlReceiver::run(){
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        
        switch(connection_type){
            case 0: //rmpc
                carInfoReceive();
                car_info_pub.publish(car_info_msg);
                break;
            case 1: //localization
                //Do nothing
                break;
            default:
                ROS_ERROR("Cannot publish the sensor msg because the connection type has been set in wrong number");
                break;
        }
        
        loop_rate.sleep();
    }
};
  
};
