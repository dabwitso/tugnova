#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <sstream>

#include <udp_msgs/UdpSensorPacket.h>
#include <carctl_msgs/monitor_status.h>

ros::Publisher pub_checkPlc;

static const int16_t INFO = 0;
static const int16_t ERROR = 1;
static const int16_t WARN = 2;

static const int DETECT_OBJECTS = 4; // 2DLiDAR障害物検知

void PlcsensorCallback(const udp_msgs::UdpSensorPacket &UdpSensorPacket)
{
  carctl_msgs::monitor_status msg;
  int error_number = UdpSensorPacket.BrakePotVol;
  std::ostringstream error_number_;
  /*
     エラー番号
    0＝No　Errors,1=緊急停止,2=バンパースイッチ,3=２Dライダー（停止）,
    4=2Dライダー（減速）,5=2Dライダー（エラー）,6=デッドマンエラー,
    7=STCエラー,8=UDP通信エラー,9=RS232C通信エラー,10=ブレーキテイクオーバー,
    11=DRエラー,12=ステアリングエラー,13=エンコーダーエラー
  */

  if (error_number == INFO)
  {
    fprintf(stdout, "DEBUG: check_plc: PLCは正常状態 \n");
    ROS_INFO("INFO PLC status ok\n");
    msg.status = INFO;
    msg.error_msg = "INFO PLC status ok";
  }
  else if (error_number != INFO)
  {
    fprintf(stdout, "DEBUG: check_plc: PLCは異常状態 errorno=[%d] \n", error_number);
    ROS_ERROR("check_plc: ERROR PLC errorno=[%d] \n", error_number);

    if (error_number == DETECT_OBJECTS) {
      msg.status = WARN;
      msg.error_msg = "plc_2_0"; // 障害発生中でも走行維持するため、エラーではなく警告で通知
    } else {
      msg.status = ERROR;
      error_number_ << error_number;
      msg.error_msg = "plc_1_" + error_number_.str();
    }
  }
  else
  {
    fprintf(stdout, "DEBUG: check_plc: 状態未取得 \n");
    ROS_ERROR("DEBUG: check_plc: Non status \n");
    return;
  }

  msg.service_name = "plc";
  pub_checkPlc.publish(msg);
  fflush(stdout);
}

int main(int argc, char **argv)
{
  fprintf(stdout, "DEBUG: check_plc start.\n");

  // ros 初期化
  ros::init(argc, argv, "check_plc");
  ros::NodeHandle n;
  // pub
  pub_checkPlc = n.advertise<carctl_msgs::monitor_status>("monitor_status", 10,true);
  // sub
  ros::Subscriber sub_plcsensor = n.subscribe("plc_sensor_packet", 10, PlcsensorCallback);
  ros::spin();

  fprintf(stdout, "DEBUG: check_plc end.\n");

  return 0;
}
