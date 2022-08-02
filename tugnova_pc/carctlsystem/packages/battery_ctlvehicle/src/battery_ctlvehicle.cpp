#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <carctl_msgs/battery_status.h>
#include <battery_checker.h>

ros::Publisher pub_BatteryStartFlg;
ros::Publisher pub_BatteryStopFlg;

static const int ON = 1;

/** バッテリ情報取得. */
void batteryStatusCallback(const carctl_msgs::battery_status &msg) {
    // バッテリ交換推奨時の停止以外は、異常発生により自動停止が行われるため、何も行う必要はないが、
    // このステータスの場合は、停止時にE1を独自にctlvehicleへ通知する必要がある。
    // バッテリ交換推奨時に走行継続を指示した場合は逆に走行開始を指示する必要がある。
    std_msgs::Int16 result;
    result.data = ON;

    switch (msg.status) {
    case BATTERY_OK:
    case BATTERY_LOW:
    case BATTERY_LOW_RUN:
    case BATTERY_EXCHANGE:
        pub_BatteryStartFlg.publish(result);
        break;
    case BATTERY_LOW_STOP:
    case BATTERY_LOW_EXCHANGE_CONFIRM:
    case BATTERY_EXCHANGE_STOP:
    case BATTERY_FATAL:
        pub_BatteryStopFlg.publish(result);
        break;
    default:
        break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "battery_ctlvehicle");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub_battery = n.subscribe("battery_status", 10, batteryStatusCallback);
    pub_BatteryStartFlg = n.advertise<std_msgs::Int16>("BatteryStartFlg", 10);
    pub_BatteryStopFlg = n.advertise<std_msgs::Int16>("BatteryStopFlg", 10);

    ros::spin();

    return 0;
}