#include <ros/ros.h>
#include <stdio.h>
#include <string.h>

namespace Udp_ns{
class UdpDirectControl{
    private:
    ros::NodeHandle nh_;

    public:
    UdpDirectControl(){
        nh_.setParam("/plc_converter/direct_control_flag", true);
        nh_.setParam("/comsctrl_converter/direct_control_flag", true);
    };
    ~UdpDirectControl(){
        nh_.setParam("/plc_converter/direct_control_flag", false);
        nh_.setParam("/comsctrl_converter/direct_control_flag", true);
    };
    void Run(){
        int cnt = 0;
        ros::Rate loop_rate(100);
        
        while(ros::ok()){
            cnt++;
            if(cnt >= 1000)
                cnt = 0;
            loop_rate.sleep();
        }
    };
};
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_direct_control");
    
    Udp_ns::UdpDirectControl udp_direct_control;
    udp_direct_control.Run();
}
