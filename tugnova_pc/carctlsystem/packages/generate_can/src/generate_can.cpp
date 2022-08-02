#define _USE_MATH_DEFINES
#include <cmath>
#include <complex>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <udp_msgs/UdpControlPacket.h>
#include <udp_msgs/UdpSensorPacket.h>
#include <ros/console.h>
#include <autoware_msgs/RemoteCmd.h>

#include "geometry_msgs/TwistStamped.h"
#include "autoware_msgs/VehicleStatus.h"

ros::Publisher vehicleStatusPub;
ros::Subscriber sub_TwistCmd;
ros::Subscriber plcSensorPacket;

// 角比率
float angle_ratio = 1.67f; // PLCから受け取ったセンサ値を実際のタイヤ角に換算する係数
// PCL使用フラグ
int _use_plc = 1;  // PLCと接続するかどうか
// ホイールベース[m]
float wheel_base = 1.14f; // 前輪と後輪の間の長さ
// 最小旋回半径[m]
float minimum_turning_radius = 1.94f; // カタログ値1.775より大きめ
// 最大操舵角[度]
float maximum_steering_angle = 60.0f; // ハードリミット

autoware_msgs::VehicleStatus vehicleStatusMsg; // Msgインスタンス作成。ros/src/msgs/autoware_msgs/msg/VehicleStatus.msgを参照

void createVehicleStatusMsg(float speed, float tire_angle_rad)
{
    std_msgs::Header header;

    header.stamp = ros::Time::now();
    header.frame_id = "base_link";

    // 必要最小限度の情報のみ生成
    vehicleStatusMsg.header = header;
    vehicleStatusMsg.speed = speed;
    vehicleStatusMsg.angle = tire_angle_rad * angle_ratio * 180.0f / M_PI;
}

void TwistCmdCallback(const geometry_msgs::TwistStamped &TwistStamped)
{
    float tire_angle_rad_ = std::asin(wheel_base * TwistStamped.twist.angular.z / TwistStamped.twist.linear.x);
    createVehicleStatusMsg(TwistStamped.twist.linear.x, tire_angle_rad_);

    ROS_DEBUG("speed:[%f]", vehicleStatusMsg.speed);
    ROS_DEBUG("angle:[%f]", vehicleStatusMsg.angle);

    if (std::isfinite(vehicleStatusMsg.speed) && std::isfinite(vehicleStatusMsg.angle)) {
        vehicleStatusPub.publish(vehicleStatusMsg);
    }
    ros::spinOnce();
}

void plcSensorPacketCallback(const udp_msgs::UdpSensorPacket &sensorMsg)
{
    // plc_sensor_packetの情報をもとに、現在車速やタイヤ角を計算し、VehicleStatusMsgを作成
    createVehicleStatusMsg((sensorMsg.PLC_vel * 3600) / 1000/* m/s -> km/h */, sensorMsg.tire_angle_rad);

    // terminalに表示
    ROS_DEBUG("speed:[%f]", vehicleStatusMsg.speed);
    ROS_DEBUG("angle:[%f]", vehicleStatusMsg.angle);

    // 車速と角度が正しく計算できたら、全体にブロードキャスト(Publish)
    // std::isfinite() -> ()内の変数が有限数であるかどうかを判定する。有限ならtrue、無限かNaNならfalse。
    if (std::isfinite(vehicleStatusMsg.speed) && std::isfinite(vehicleStatusMsg.angle)) {
        vehicleStatusPub.publish(vehicleStatusMsg);
    }
    ros::spinOnce();
}


int main(int argc, char **argv)
{
    ROS_DEBUG("generate_can start");

    ros::init(argc, argv, "generate_can");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    // ros parameter settings
    if (n.hasParam("/vehicle_info/wheel_base") && n.hasParam("/vehicle_info/minimum_turning_radius") &&
        n.hasParam("/vehicle_info/maximum_steering_angle"))
    {
        n.getParam("/vehicle_info/wheel_base", wheel_base);
        ROS_DEBUG("wheel_base[%f]", wheel_base);

        n.getParam("/vehicle_info/minimum_turning_radius", minimum_turning_radius);
        ROS_DEBUG("minimum_turning_radius[%f]", minimum_turning_radius);

        n.getParam("/vehicle_info/maximum_steering_angle", maximum_steering_angle);
        ROS_DEBUG("maximum_steering_angle[%f]", maximum_steering_angle);

        angle_ratio = maximum_steering_angle / (std::asin(wheel_base / minimum_turning_radius) * 180.0f / M_PI);
        ROS_DEBUG("angle_ratio[%f]", angle_ratio);
    }

    if (private_nh.hasParam("use_plc"))
    {
        private_nh.getParam("use_plc", _use_plc);
    }
    ROS_DEBUG("_use_plc[%d]", _use_plc);

    // メインプロセス
    // PLCを使っているならplc_sensor_packetトピックの値を使用して、VehicleStatusMsgを作成、Publishする(ros::spin()以降で）。
    if (_use_plc == 0) {
        sub_TwistCmd = n.subscribe("twist_cmd", 10, TwistCmdCallback);
    } else {
        plcSensorPacket = n.subscribe("/plc_sensor_packet", 10, plcSensorPacketCallback);
    }

    // 作成したVehicleStatusMsgを全体にブロードキャスト（Publish）するインスタンス？のようなものを作成。
    // これ以降は、vehicleStatusPub.publishでPublishできるようになる。
    vehicleStatusPub = n.advertise<autoware_msgs::VehicleStatus>("/vehicle_status", 10);

    // 以上でこのノードは確立(establish)される。Callbackの対象がPublishされるたびに、Callback関数が実行される。
    ros::spin();
    // これ以降はCtrl+Cまで実行されない。
    ROS_DEBUG("generate_can end");
    return 0;
}
