#ifndef COMSCTRL_CONNECTOR_H_
#define COMSCTRL_CONNECTOR_H_

#include <udp_connector.h>

#pragma pack(1)
#include <udp_msgs/CarInfo.h>
#include <udp_msgs/ControlInfo.h>
#include <udp_msgs/LocalizationInfo.h>
#pragma pack()

#define BUFLEN 512

namespace Udp_ns{
class ComsctrlSender : public UdpSender{
public:
    ComsctrlSender(std::string _destination, unsigned short _port, int _connection_type);
    ~ComsctrlSender();
    void run();
    
private:
    //Node handles
    ros::NodeHandle nh;

    //-------------------To send------------------------//
    //ControlInfo
    ros::Subscriber control_info_sub;
    udp_msgs::ControlInfo control_info_msg;
    void controlInfoCallback(const udp_msgs::ControlInfo& msg);
    void controlInfoSend(const udp_msgs::ControlInfo& msg);
    
    //LocalizationInfo
    ros::Subscriber localization_info_sub;
    udp_msgs::LocalizationInfo localization_info_msg;
    void localizationInfoCallback(const udp_msgs::LocalizationInfo& msg);
    void localizationInfoSend(const udp_msgs::LocalizationInfo& msg);

    int connection_type; //0: rmpc, 1: localization, -1: else (error)
};

class ComsctrlReceiver : public UdpReceiver{
public:
    ComsctrlReceiver(unsigned short _port, int _connection_type);
    ~ComsctrlReceiver();
    void run();
    
private:
    //Node handles
    ros::NodeHandle nh;
    
    //-------------------To receive----------------------//
    //CarInfo
    ros::Publisher car_info_pub;
    udp_msgs::CarInfo car_info_msg;
    void carInfoReceive();

    int connection_type; //0: rmpc, 1: localization, -1: else (error)
};

enum CarInfoMode{
    X = 0,
    Y = 1,
    VELOCITY = 2,
    YAW = 3,
    TIRE_ANGLE = 4,
    DATACKR = 5,
};

};

#endif //COMSCTRL_CONNECTOR_H_
