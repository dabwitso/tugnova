#include <ros/ros.h>

#include <carctl_msgs/battery_scale.h>
#include <carctl_msgs/battery_status.h>
#include <carctl_msgs/battery_checker_command.h>
#include <autoware_msgs/Lane.h>
#include <carctl_msgs/monitor_status.h>
#include <std_msgs/Int16.h>

#include <battery_checker.h>

int scale_range; // 最大目盛数（6, 10, 20のいずれか）
int scale_failure; // 電圧異常検知用 閾値

ros::Publisher pub_status;
ros::Publisher pub_monitor;

int current_waypoint_id = 0;
int current_battery_exchange = 0;
int current_battery_exchange_threshold = 0;
int current_battery_low = 0;
int current_battery_low_threshold = 0;

int prev_waypoint_id = 0;
int scale = 0; // 目盛(1～20)
int prev_color = COLOR_NORMAL;
int status = BATTERY_OK;

const int INFO = 0;
const int ERROR = 1;
const int WARN = 2;

const std::string LOW_MSG                  = "battery_2_0";
const std::string LOW_STOP_MSG             = "battery_2_1";
const std::string EXCHANGE_MSG             = "battery_2_2";
const std::string BLOCK_FORCE_START_MSG    = "battery_2_3";
const std::string LOW_EXCHANGE_CONFIRM_MSG = "battery_2_4";
const std::string EXCHANGE_STOP_MSG        = "battery_1_0";
const std::string ERROR_MSG                = "battery_1_1";

int monitor_status = INFO;
std::string monitor_message = "";

const int COMMAND_INIT = 0;
const int COMMAND_RUN = 1;
const int COMMAND_EXCHANGE = 2;

const int GPIO_ON = 1;

/** battery_status送信. */
void publishBatteryStatus(const int _status) {
    status = _status; // 他でも使うため、状態を記録する

    int color;
    switch (status) {
    case BATTERY_LOW:
    case BATTERY_LOW_STOP:
        color = COLOR_WARNING;
        break;
    case BATTERY_EXCHANGE:
    case BATTERY_EXCHANGE_STOP:
        color = COLOR_ERROR;
        break;
    case BATTERY_FATAL:
        color = COLOR_FATAL;
        break;
    default:
        // 上記以外のステータスの場合は、過去の背景色を引き継ぐ
        color = prev_color;
    }
    prev_color = color;

    carctl_msgs::battery_status msg;
    msg.status = status;
    msg.scale = scale;
    msg.color = color;
    pub_status.publish(msg);
}

/** monitor_status送信. */
void publishMonitorStatus(const int status, const std::string message) {
    carctl_msgs::monitor_status msg;
    msg.service_name = "battery";
    msg.status = monitor_status = status;
    msg.error_msg = monitor_message = message;
    pub_monitor.publish(msg);
}

/** 目盛情報の受信. */
void scaleCallback(const carctl_msgs::battery_scale &msg) {
    scale = (msg.scale >= scale_range) ? scale_range : (msg.scale <= 0) ? 0 : msg.scale;
}

/** 画面からの命令受信. */
void commandCallback(const carctl_msgs::battery_checker_command &msg) {
    const int command = (msg.command > COMMAND_EXCHANGE || msg.command < COMMAND_INIT) ? COMMAND_INIT : msg.command;

    if (status != BATTERY_LOW_STOP) {
        ROS_WARN("Reject Command.");
        return;
    }

    switch (command) {
    case COMMAND_RUN:
        publishBatteryStatus(BATTERY_LOW_RUN);
        publishMonitorStatus(INFO, ""); // 警告メッセージの解除
        ROS_INFO("Accept Command RUN.");
        break;
    case COMMAND_EXCHANGE:
        publishBatteryStatus(BATTERY_LOW_EXCHANGE_CONFIRM);
        publishMonitorStatus(WARN, LOW_EXCHANGE_CONFIRM_MSG);
        ROS_INFO("Accept Command EXCHANGE.");
        break;
    default:
        ROS_ERROR("Unknown Command.");
        break;
    }
}

/** GPIOボタン押下時. */
void gpioStartFlgCallback(const std_msgs::Int16 &msg) {
    if (status != BATTERY_LOW_STOP || msg.data != GPIO_ON) {
        ROS_WARN("Reject Gpio.");
        return;
    }

    // メッセージのみ変更
    publishMonitorStatus(WARN, BLOCK_FORCE_START_MSG);
}

/** バッテリ要交換か検証. (優先して検証) */
bool checkBatteryExchange(const int battery_exchange, const int threshold) {
    switch (battery_exchange) {
    case CHECK_AND_STOP:
        if (scale > threshold) { break; }
        publishBatteryStatus(BATTERY_EXCHANGE_STOP);
        publishMonitorStatus(ERROR, EXCHANGE_STOP_MSG);
        ROS_INFO("Stop. (battery_exchange)");
        return false;
    case CHECK_ONLY:
        if (scale > threshold) { break; }
        publishBatteryStatus(BATTERY_EXCHANGE);
        publishMonitorStatus(WARN, EXCHANGE_MSG);
        ROS_INFO("Detect. (battery_exchange)");
        return false;
    case STOP_ONLY:
        if (status != BATTERY_EXCHANGE) {
            ROS_WARN("Cannot Stop Before Detect. (battery_exchange)");
            break;
        }
        publishBatteryStatus(BATTERY_EXCHANGE_STOP);
        publishMonitorStatus(ERROR, EXCHANGE_STOP_MSG);
        ROS_INFO("Stop. (battery_exchange)");
        return false;
    default:
        break; // 検知や停止位置ではない限りはこちらに入る
    }

    return true;
}

/** バッテリ交換推奨警告か検証. */
bool checkBatteryLow(const int battery_low, const int threshold) {
    switch (battery_low) {
    case CHECK_AND_STOP:
        if (scale > threshold) { break; }
        publishBatteryStatus(BATTERY_LOW_STOP);
        publishMonitorStatus(WARN, LOW_STOP_MSG);
        ROS_INFO("Stop. (battery_low)");
        return false;
    case CHECK_ONLY:
        if (scale > threshold) { break; }
        publishBatteryStatus(BATTERY_LOW);
        publishMonitorStatus(WARN, LOW_MSG);
        ROS_INFO("Detect. (battery_low)");
        return false;
    case STOP_ONLY:
        if (status != BATTERY_LOW) {
            ROS_WARN("Cannot Stop Before Detect. (battery_low)");
            break;
        }
        publishBatteryStatus(BATTERY_LOW_STOP);
        publishMonitorStatus(WARN, LOW_STOP_MSG);
        ROS_INFO("Stop. (battery_low)");
        return false;
    default:
        break; // 検知や停止位置ではない限りはこちらに入る
    }

    return true;
}

/** safety_waypoints受信. */
void waypointCallback(const autoware_msgs::Lane &msg) {
    const autoware_msgs::WaypointCustom wpc = msg.waypoints[0].wpc;
    current_waypoint_id = wpc.waypoint_id;
    current_battery_exchange = wpc.battery_exchange;
    current_battery_exchange_threshold = wpc.battery_exchange_threshold;
    current_battery_low = wpc.battery_low;
    current_battery_low_threshold = wpc.battery_low_threshold;
}

/** 更新処理. */
void update() {
    // battery_converterでも対策はしているが、こちらでも対策をいれる. 目盛0以下の場合は判定処理をしない.
    // ctlvehicleのBatteryStartFlg初期状態の解除のためバッテリは正常であると送信. 解除しないと発進を指示できない.
    if (scale <= 0) {
        publishBatteryStatus(BATTERY_OK);
        return;
    }

    // 正常以外の場合、移動しきっていない状態での再チェックは行わず、同じステータス、メッセージを再送する
    if (status != BATTERY_OK && current_waypoint_id == prev_waypoint_id) {
        publishBatteryStatus(status);
        publishMonitorStatus(monitor_status, monitor_message);
        return;
    }
    prev_waypoint_id = current_waypoint_id;

    // 電圧異常時は即時停止
    if (scale <= scale_failure) {
        publishBatteryStatus(BATTERY_FATAL);
        publishMonitorStatus(ERROR, ERROR_MSG);
        ROS_INFO("Stop. (battery_dead)");
        return;
    }

    // 交換推奨、要交換の検知は指定地点でのみ対応
    if (!checkBatteryExchange(current_battery_exchange, current_battery_exchange_threshold)) {
        return;
    }

    if (!checkBatteryLow(current_battery_low, current_battery_low_threshold)) {
        return;
    }

    // 何らかのステータス異常を検知した後、移動した場合、それを繰り返す
    // ただし、バッテリ警告後に発進指示を出した後は、次の検知位置まで行くまでは正常と同じ扱いで処理する.
    if (status == BATTERY_OK || status == BATTERY_LOW_RUN) {
        publishBatteryStatus(BATTERY_OK);
    } else {
        publishBatteryStatus(status);
        publishMonitorStatus(monitor_status, monitor_message);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "battery_checker");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("scale_range", scale_range, 10); // 6 or 10 or 20
    private_nh.param<int>("scale_failure", scale_failure, 2);

    pub_status = n.advertise<carctl_msgs::battery_status>("battery_status", 10, true);
    pub_monitor = n.advertise<carctl_msgs::monitor_status>("monitor_status", 10);

    ros::Subscriber sub_scale = n.subscribe("battery_scale", 10, scaleCallback);
    ros::Subscriber sub_command = n.subscribe("battery_checker_command", 10, commandCallback);
    ros::Subscriber sub_waypoint = n.subscribe("safety_waypoints", 10, waypointCallback);
    ros::Subscriber sub_gpio = n.subscribe("GpioStartFlg", 10, gpioStartFlgCallback);

    ros::Rate rate(10.0);
    while (ros::ok()) {
        ros::spinOnce();
        update();
        rate.sleep();
    }

    return 0;
}