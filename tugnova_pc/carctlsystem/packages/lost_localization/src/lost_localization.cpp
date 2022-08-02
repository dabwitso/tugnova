#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <autoware_msgs/RemoteCmd.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <carctl_msgs/monitor_status.h>
#include <carctl_msgs/emergency_status.h>
#include <udp_msgs/UdpSensorPacket.h>

ros::Publisher statecmd;
ros::Publisher pub_emergency_error;

//自動走行状態
const int16_t AUTO_RUN = 1;
int16_t runMode = AUTO_RUN;

//異常状態
const int16_t NORMAL = 0;
const int16_t ERROR = 1;
const int16_t WARN = 2;
int16_t ndt_monitor_status = NORMAL;

const std::string LOST_LOCALIZATION = "lost_localization";
const std::string MAIGO = "lost_localization_1_0";
const std::string DASSEN = "lost_localization_1_1";
const std::string KEIROGAI = "lost_localization_1_2";

// 脱線系エラーの判定
std::string emergency = "";

// 自動運転中か
bool isAutodrive = false;

void publishMonitorStatus(int16_t status, std::string error_msg)
{
  carctl_msgs::monitor_status msg;
  //ノード名は固定
  msg.service_name = LOST_LOCALIZATION;
  //状態を格納
  msg.status = status;
  //エラーメッセージを格納
  msg.error_msg = error_msg;

  carctl_msgs::emergency_status emergency_status;
  emergency_status.service_name = LOST_LOCALIZATION;
  emergency_status.status = NORMAL;

  if (status == ERROR)
  {
    //エラーログ出力
    ROS_ERROR("Infomation: [%s] \n", error_msg.c_str());
    emergency_status.status = ERROR;
  }

  //パブリッシュ
  statecmd.publish(msg);
  pub_emergency_error.publish(emergency_status);

  ros::spinOnce();
}

void remoteCmdCallback(const autoware_msgs::RemoteCmd remote_cmd)
{
  if (emergency != "") { return; } // 勝手な復帰を防ぐため、念のためブロックする
  if (remote_cmd.control_mode != 4 && !isAutodrive) { // 4 = scan_respawn_pointスクリプトからの通知は画面UIのエラー状態維持のため無条件で脱線エラーとする
    return; // 手動運転中のエラー検知を防止する
  }

  if (remote_cmd.vehicle_cmd.emergency) {
    switch (remote_cmd.control_mode) {
      case 0: emergency = DASSEN; break;
      case 3: // through
      case 4: emergency = KEIROGAI; break;
      default: /* Use 1 and 2 in tiwst_get. */
        break;
    }
  }
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose)
{
  emergency = "";
  ndt_monitor_status = NORMAL;
  ROS_INFO("emergency: Clear");
}

void initialPose2Callback(const geometry_msgs::PoseWithCovarianceStamped pose)
{
  emergency = "";
  ndt_monitor_status = NORMAL;
  ROS_INFO("emergency: Clear");
}

void ndtStatusCallback(const std_msgs::String ndt_status_msg)
{
  if (ndt_monitor_status != ERROR) { // 勝手な復元を防ぐ
    if (ndt_status_msg.data == "NDT_ERROR" || ndt_status_msg.data == "NDT_FATAL") {
      ndt_monitor_status = ERROR;
    } else if (ndt_status_msg.data == "NDT_WARNING") {
      ndt_monitor_status = WARN;
    } else {
      ndt_monitor_status = NORMAL;
    }
  }

  if (ndt_monitor_status == ERROR) {
    ROS_ERROR("ndtStatusCallback MAIGO ERROR");
    publishMonitorStatus(ERROR, MAIGO);
  } else if (emergency != "") { // 迷子の時は必ず脱線するため、エラーは迷子を優先とする
    ROS_ERROR("ndtStatusCallback %s ERROR", emergency.c_str());
    publishMonitorStatus(ERROR, emergency);
  } else if (ndt_monitor_status == WARN) {
    ROS_WARN("ndtStatusCallback WARNING");
    publishMonitorStatus(WARN, "");
  } else {
    ROS_INFO("ndtStatusCallback NORMAL");
    publishMonitorStatus(NORMAL, "");
  }
}

void plcSensorCallback(const udp_msgs::UdpSensorPacket::ConstPtr& msg) {
  isAutodrive = (msg->ECUMode == 0) ? false : true;
}

int main(int argc, char **argv)
{
  ROS_INFO("lost_localization start.");

  // ros 初期化
  ros::init(argc, argv, "lost_localization");
  ros::NodeHandle n;
  // pub
  statecmd = n.advertise<carctl_msgs::monitor_status>("/monitor_status", 10);
  pub_emergency_error = n.advertise<carctl_msgs::emergency_status>("/emergency_error", 10);
  // sub
  ros::Subscriber remoteCmd = n.subscribe("remote_cmd", 10, remoteCmdCallback);
  ros::Subscriber ndtStatus = n.subscribe("/ndt_monitor/ndt_status", 10, ndtStatusCallback);
  ros::Subscriber plcSensor = n.subscribe("/plc_sensor_packet", 10, plcSensorCallback);
  ros::Subscriber initpose = n.subscribe("/initialpose", 10, initialPoseCallback);
  ros::Subscriber initpose2 = n.subscribe("/initialpose2", 10, initialPose2Callback);

  // メインループ開始
  ros::spin();

  ROS_INFO("lost_localization end.");

  return 0;
}
