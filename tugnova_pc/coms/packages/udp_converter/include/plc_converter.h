#ifndef PLC_CONVERTER_H_
#define PLC_CONVERTER_H_

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/AccelCmd.h>
#include <autoware_msgs/SteerCmd.h>
#include <autoware_msgs/BrakeCmd.h>
#include <autoware_msgs/LampCmd.h>
#include <autoware_msgs/IndicatorCmd.h>
#include <autoware_msgs/LaneArray.h>
#include <tablet_socket_msgs/gear_cmd.h>
#include <tablet_socket_msgs/mode_cmd.h>
#include <carctl_msgs/emergency_status.h>
#include <carctl_msgs/monitor_status.h>

#pragma pack(1)
#include <udp_msgs/UdpControlPacket.h>
#include <udp_msgs/UdpSensorPacket.h>
#include <udp_msgs/UdpSensorPacket_GreenComs.h>
#pragma pack()

#define MIN_COUNT (0)
#define MAX_COUNT (65535)
#define THRESHOLD ((MAX_COUNT-MIN_COUNT)/2.0)
#define mps2kmph(x) (x*3.6)
#define MAX_DIFF_TIME (20e-03)

#define STRING(var) #var

//車両挙動指示として送信する値。
//車両挙動指示を下記のルールで変換した数値をPLCに送る
//一桁目をアルファベットの順番にする
//二桁目の数値はそのまま
//例）A1 →　1(Aのアルファベットの順番)1(2桁目はそのまま)
#define A1 (11)
#define D2 (42)
#define E1 (51)
#define E2 (52)
#define EMERGENCY_INIT (0)

#define ON (1)
#define OFF (0)
#define NO_DETECTION (0)
#define DETECTION (1)


namespace Udp_ns{

class lowpassFilter
{
public:
    void setSize(int sizein);
    lowpassFilter();
    lowpassFilter(int sizein);
    void setFrequency(double cutoffFrequency, double samplingFrequency);
    double compute();
    void savePreviousInput(double xnew);

private:
    std::vector<double> x;
    std::vector<double> y;
    int size;
    double b0, b1, b2;
    double a0, a1, a2;
};


class medianFilter
{
public:
    void setSize(int sizein);
    medianFilter();
    medianFilter(int sizein);
    double compute();
    void savePreviousInput(double xnew);
  
private:
  std::vector<double> x;
  std::vector<double> y;
  int size;
};


enum ComsMode{
    GREEN = 0,
    YUKURI = 1,
};


class PlcConverter{
public:
    PlcConverter();
    void run();
    void publishMsgs();
    double calc_time();
    bool check_topic_receive_time(double topic_receive_time);
    double publish_emergency_error(int16_t status);
    double publishMonitorStatus(int16_t status, std::string error_msg);
    
    
private:
    ros::Time time;
    ros::Time previous_time;
    ros::Time udp_sensor_packet_updated_time;

    //Node handles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    
    //Flag
    bool udp_direct_control_flag;
    bool use_low_pass_filter;
    bool use_median_filter;

    int veicle_status;
    // 前回速度
    float last_twistx;
    ros::WallTime nowtime;
    ros::WallTime runtime;
    ros::WallTime stoptime;
    int32_t waittime;

    //Coms
    Udp_ns::ComsMode coms;
    std::string coms_str;
    float accelGain;
    float brakeGain;

    int16_t emergency_stop_state;
    int16_t is_detection_lidar;
    
    double receive_error_threshold;
    double twist_cmd_topic_receive_time;
    int16_t srv_start_flg;
    int16_t pouse_start_flg;
    int16_t connect_start_flg;
    int16_t battery_start_flg;
    int16_t cancel_possible_emergency_error_by_scan;
    const int16_t RUN = 0;
    const int16_t STOP = 1;
    const int16_t NOT_OCCURRED = 1;
    const int16_t OCCURRED = 0;
    const int16_t INFO = 0;
    const int16_t ERROR = 1;

    const std::string NOMAL_ID = "plc_converter_0_0";
    const std::string ERROR_ID = "plc_converter_1_0";
    
    //Subscribers and Publishers
    //Sensor packet -> twist, pose
    ros::Subscriber udp_sensor_packet_sub;
        
    ros::Publisher curr_twist_pub;
    ros::Publisher curr_pose_pub;
    
    //Cmd -> control packet
    ros::Subscriber twist_cmd_sub;
    ros::Subscriber AccelCmd_sub;
    ros::Subscriber SteerCmd_sub;
    ros::Subscriber BrakeCmd_sub;
    // ros::Subscriber LampCmd_sub;
    ros::Subscriber IndicatorCmd_sub;
    ros::Subscriber gear_cmd_sub;
    ros::Subscriber mode_cmd_sub;
    ros::Subscriber plc_errormsg_sub;
    ros::Subscriber waypoint_sub;
    ros::Subscriber liadr_vel_sub; // 20200908@add : golf_cart driver
    ros::Subscriber stopflg_sub; // 20210209@add : acc_frc_ref (running Botton)

    ros::Subscriber emergency_stop_sub;
    ros::Subscriber sub_SrvStart;
    ros::Subscriber sub_PouseStart;
    ros::Subscriber sub_ConnectStart;
    ros::Subscriber sub_BatteryStart;
    ros::Publisher pub_emergency_error;
    ros::Publisher pub_monitor_status;
    ros::Subscriber sub_scan_respawn_result;
    ros::Subscriber sub_monitor_status;

    ros::Publisher udp_control_packet_pub;
    
    //Topic names
    std::string twist_cmd_topic;
    std::string AccelCmd_topic;
    std::string SteerCmd_topic;
    std::string BrakeCmd_topic;
    // std::string LampCmd_topic;
    std::string IndicatorCmd_topic;
    std::string gear_cmd_topic;
    std::string mode_cmd_topic;
    std::string plc_errormsg_topic;
    std::string waypoint_topic;
    std::string lidar_vel_topic; // 20200908@add : golf_cart driver
    std::string stopflg_topic; // 20210209@add : acc_frc_ref (running Botton)

    //Topic messages
    udp_msgs::UdpSensorPacket udp_sensor_packet_msg;
    udp_msgs::UdpSensorPacket udp_sensor_packet_msg_previous;
    udp_msgs::UdpSensorPacket_GreenComs udp_sensor_packet_green_coms_msg;
    udp_msgs::UdpSensorPacket_GreenComs udp_sensor_packet_green_coms_msg_previous;
    
    geometry_msgs::TwistStamped curr_twist_msg;
    geometry_msgs::PoseStamped curr_pose_msg;

    udp_msgs::UdpControlPacket udp_control_packet_msg;


    //Filters
    lowpassFilter lowpass_wr;
    lowpassFilter lowpass_wl;
    medianFilter median_wr;
    medianFilter median_wl;

    //Filter parameters
    double median_filter_size_wr;
    double median_filter_size_wl;
    double lowpass_filter_cutoff_frequency_wr;
    double lowpass_filter_cutoff_frequency_wl;
    double lowpass_filter_sampling_frequency_wr;
    double lowpass_filter_sampling_frequency_wl;
    double lowpass_filter_size_wr;
    double lowpass_filter_size_wl;
    double encoder_pulse_resolution;

    // vehicle information
    double wheel_base;
    double minimum_turning_radius;
    double maximum_steering_angle;
    double tire_radius;
    double tire_distance;

    //Filtering function
    void computeEncoderSpeed( double encoderPulseResolution, double tireRadius, double tireDistance, bool useLowpassFilter, bool useMedianFilter );

    double clipValue(double input, double min, double max);
    float clipValue(float input, float min, float max);
    int clipValue(int input, int min, int max);

    void drivingValueControl(float acc_vel_ref_,float angularz_,int brk_cmode,int acc_pos_ref);

    //Callback functions
    void udpSensorPacketCallback(const udp_msgs::UdpSensorPacket& msg);
    void udpSensorPacketGreenComsCallback(const udp_msgs::UdpSensorPacket_GreenComs& msg);
    void twistCmdCallback(const geometry_msgs::TwistStamped& msg);
    void accelCmdCallback(const autoware_msgs::AccelCmd& msg);
    void steerCmdCallback(const autoware_msgs::SteerCmd& msg);
    void brakeCmdCallback(const autoware_msgs::BrakeCmd& msg);
    // void lampCmdCallback(const autoware_msgs::LampCmd& msg);
    void indicatorCmdCallback(const autoware_msgs::IndicatorCmd& msg);
    void gearCmdCallback(const tablet_socket_msgs::gear_cmd& msg);
    void modeCmdCallback(const tablet_socket_msgs::mode_cmd& msg);
    void plcErrormsgCallback(const std_msgs::Int16& msg);
    void waypointCallback(const autoware_msgs::Lane& msg);
    void lidarvelCallback(const geometry_msgs::TwistStamped& msg); // 20200908@add : golf_cart driver
    void stopflgCallback(const std_msgs::Int16& msg); // 20210209@add : acc_frc_ref (running Botton)

    void EmergencyStopCallback(const std_msgs::Int16 &msg);
    void SrvStartCallBack(const std_msgs::Int16 &flg);
    void PouseStartCallBack(const std_msgs::Int16 &flg);
    void ConnectStartCallBack(const std_msgs::Int16 &flg);
    void BatteryStartCallBack(const std_msgs::Int16 &flg);
    void monitorstatusCallback(const carctl_msgs::monitor_status &m_status);
    void scanRespawnResultCallback(const std_msgs::Int16 msg);
};

}
#endif
