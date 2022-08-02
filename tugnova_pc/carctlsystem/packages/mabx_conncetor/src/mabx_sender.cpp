#include <mabx_connector.h>

namespace Mabx {

    MabxSender::MabxSender(std::string _destination, unsigned short _port) : UdpSender(_destination, _port) {
        udp_control_packet_sub = nh.subscribe("/plc_control_packet", 1, &Mabx::MabxSender::udpControlPacketCallback, this);
        udp_sensor_packet_sub = nh.subscribe("/plc_sensor_packet", 1, &Mabx::MabxSender::udpSensorPacketCallback, this);
    };

    MabxSender::~MabxSender() {
        close(socket_var);
    };

    /**
     * PLCコントールパケットの記録.
     */
    void MabxSender::udpControlPacketCallback(const udp_msgs::UdpControlPacket& msg) {
        udp_control_packet_msg = msg;
    };

    /**
     * PLCセンサーパケットの記録.
     */
    void MabxSender::udpSensorPacketCallback(const udp_msgs::UdpSensorPacket& msg) {
        udp_sensor_packet_msg = msg;
    };

    /**
     * MABX宛てのUDPパケット生成.
     */
    carctl_msgs::MabxSenderPacket MabxSender::createMsg(
        const udp_msgs::UdpControlPacket& control_msg, const udp_msgs::UdpSensorPacket& sensor_msg) {
        carctl_msgs::MabxSenderPacket msg;
        msg.auto_control = control_msg.auto_control;
        msg.acc_cmode = control_msg.acc_cmode;
        msg.acc_vel_ref = control_msg.acc_vel_ref;
        msg.tir_ang_ref = control_msg.tir_ang_ref;
        msg.winkerL = control_msg.winkerL;
        msg.winkerR = control_msg.winkerR;
        msg.tire_angle_rad = sensor_msg.tire_angle_rad;
        msg.PLC_vel = sensor_msg.PLC_vel;
        msg.wd_count = control_msg.wd_count;
        return msg;
    }

    /**
     * MABXへのデータ送信部.
     */
    void MabxSender::udpPacketSend(const carctl_msgs::MabxSenderPacket msg) {
        int packet_length = sizeof(msg);
        if( sendto(socket_var, &msg, packet_length, 0, (const sockaddr*)&sock_addr, sizeof(sock_addr)) < 0 )
        {
            ROS_ERROR("cannot send packet" );
        }
    }

    /**
     * メイン関数.
     */
    void MabxSender::run() {
        ros::Rate loop_rate(100);
        while(ros::ok()){
            ros::spinOnce();

            udpPacketSend(createMsg(udp_control_packet_msg, udp_sensor_packet_msg));

            loop_rate.sleep();
        }
    };
};