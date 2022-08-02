#include <ros/ros.h>
#include <map>
#include <chrono>
#include <cmath>

#include <udp_msgs/UdpSensorPacket.h>
#include <carctl_msgs/battery_scale.h>

std::string mode; // モード(battery or soc)
double maximum_voltage; // 満充電時の電圧
double scale_threshold; // 目盛下げるための定数
int i_scale_range;
double scale_range; // 最大目盛数（6, 10, 20のいずれか）
double timer; // 目盛確定までの秒数
double initial_timer; // 起動直後の目盛判断までの秒数

ros::Publisher pub_scale;
int fixed_scale; // 確定した目盛
int prev_scale = -1; // （ログ出力用）目盛が変化した時のみログを出力する。初回は必ず出したいので-1スタート

std::chrono::system_clock::time_point prev_time; // 前回の確認時刻
double total_time; // バッテリ低下中の合計時間

std::chrono::system_clock::time_point startup_time; // 起動時間
std::vector<double> startup_volts;
bool has_initial_scale = false;

int timer_for_test = 1; // 検証時のみ利用するタイマー(秒)
std::chrono::system_clock::time_point timerpoint_for_test; // 検証時のみ利用するタイマー

/** 電圧を目盛に変換. */
int convertScale(const double voltage) {
    double start = maximum_voltage;
    double end   = maximum_voltage - scale_threshold;
    int scale = i_scale_range;

    if (voltage >= maximum_voltage) {
        return scale; // 通常ありえないが、満充電時を上回っていた場合は最大値の目盛を返す
    }

    for (; scale >= 1; scale--) {
        if (voltage <= start && voltage > end) {
            return scale;
        }
        start = end;
        end -= scale_threshold;
    }

    return 1; // 通常到達しないが、到達した場合は最低値の目盛を返す
}

/** 起動から一定時間経過後に平均Voltを用いて初期目盛を確定する. */
int outputInitialScale(const double voltage, const std::chrono::system_clock::time_point now) {
    std::chrono::duration<double> diff = now - startup_time;
    startup_volts.push_back(voltage);

    if (std::chrono::duration_cast<std::chrono::microseconds>(diff).count() > initial_timer) {
        has_initial_scale = true;
        prev_time = now;
        fixed_scale = convertScale(std::accumulate(startup_volts.begin(), startup_volts.end(), 0.0) / startup_volts.size());
    } else {
        fixed_scale = 0;
    }

    return fixed_scale;
}

/** Btry_V(電圧入力)を目盛へ変換. */
int modeBattery(const double voltage) {
    if (voltage <= 0) { return 0; }

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    if (!has_initial_scale) {
        return outputInitialScale(voltage, now);
    }

    const int temp_scale = convertScale(voltage);

    if (fixed_scale > temp_scale) {
        std::chrono::duration<double> diff = now - prev_time;
        total_time += std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
    }
    prev_time = now;

    if (total_time >= timer) {
        fixed_scale = (fixed_scale <= 1) ? 1 : (fixed_scale-1);
        total_time = 0.0;
    }

    return fixed_scale;
}

/** SOC(パーセンテージ入力)を目盛へ変換. */
int modeSOC(const int percent) {
    if (percent <= 0) { return 0; }
    return std::ceil(static_cast<double>(percent) / (100.0 / scale_range));
}

/** battery_scaleの送信. */
void publishBatteryScaleMsg(const int scale) {
    carctl_msgs::battery_scale msg;
    msg.scale = scale;
    pub_scale.publish(msg);
}

/** plc_sensor_packet受信時の処理. */
void plcSensorPacketCallback(const udp_msgs::UdpSensorPacket &udpSensorPacket) {
    int scale;
    if (mode == "battery") {
        scale = modeBattery(udpSensorPacket.Btry_V);
    } else if (mode == "soc") {
        scale = modeSOC(udpSensorPacket.CART_SOC);
    } else {
        scale = 0;
    }

    // 検証時用のログ出力
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = now - timerpoint_for_test;
    if (std::chrono::duration_cast<std::chrono::microseconds>(diff).count() > (timer_for_test * 1000 * 1000)) {
        timerpoint_for_test = now;
        ROS_INFO("scale: %d, voltage: %f, soc: %d", scale, udpSensorPacket.Btry_V, udpSensorPacket.CART_SOC);
    }

    if (scale <= 0) {
        publishBatteryScaleMsg(0);
        return;
    }

    if (prev_scale != scale) {
        ROS_INFO("scale: %d, voltage: %f, soc: %d", scale, udpSensorPacket.Btry_V, udpSensorPacket.CART_SOC);
    }
    prev_scale = scale;

    publishBatteryScaleMsg(scale);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "battery_converter");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("mode", mode, "battery"); // battery or soc
    private_nh.param<double>("maximum_voltage", maximum_voltage, 100.0);
    private_nh.param<double>("scale_threshold", scale_threshold, 10.0);

    private_nh.param<int>("scale_range", i_scale_range, 10); // 6 or 10 or 20
    if (i_scale_range != 6 && i_scale_range != 10 && i_scale_range != 20) { i_scale_range = 20; }
    scale_range = static_cast<double>(i_scale_range);

    int timer_;
    private_nh.param<int>("timer", timer_, 10);
    timer = static_cast<double>(timer_ * 1000 * 1000);

    int initial_timer_;
    private_nh.param<int>("initial_timer", initial_timer_, 60);
    initial_timer = static_cast<double>(initial_timer_ * 1000 * 1000);
    startup_time = std::chrono::system_clock::now();

    pub_scale = n.advertise<carctl_msgs::battery_scale>("battery_scale", 10, true);
    ros::Subscriber sub_plcsensor = n.subscribe("plc_sensor_packet", 10, plcSensorPacketCallback);

    ros::spin();

    return 0;
}