#include <ros/ros.h>
#include <sstream>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "autoware_msgs/LaneArray.h"
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <carctl_msgs/monitor_status.h>
#include <udp_msgs/UdpControlPacket.h>
#include <udp_msgs/UdpSensorPacket.h>

static const int16_t NORMAL = 0;
static const int16_t ERROR = 1;
static const int16_t WARN = 2;

//障害物検知以外異常が発生しているか
int16_t is_error_ = NORMAL;

//plcエラーの場合に2dセンサーエラー（障害物検知）とその他のplcエラーを見分ける必要があるため、変数を２つ用意する
//2dセンサーエラー以外のplc異常が発生しているか
int16_t is_plc_error_ = NORMAL;

static const int16_t STOP = 0;
static const int16_t RUN = 1;

//走行中か
int16_t is_run_ = RUN;

static const int16_t STRAIGHT = 0;
static const int16_t LEFT = 1;
static const int16_t RIGHT = 2;
//ウィンカー
int16_t turn_signal_state_ = STRAIGHT;

//lidarと2Dセンサーの物体検知は同時に発生する可能性があるため、変数を2つ用意する
static const int16_t NO_DETECTION = 0;
static const int16_t DETECTION = 1;
//lidarで物体検知しているか
int16_t is_detection_lidar_ = NO_DETECTION;
//2Dセンサーで物体検知しているか
int16_t is_detection_2d_ = NO_DETECTION;

std::vector<std::string> error_list;

//LEDの表示を変更するためのコマンド
const char *auto_cmd = "/home/nvidia/Autoware/ros/nodeshell/LED_auto.sh";
const char *stop_cmd = "/home/nvidia/Autoware/ros/nodeshell/LED_stop.sh";
const char *error_cmd = "/home/nvidia/Autoware/ros/nodeshell/LED_error.sh";
const char *detection_cmd = "/home/nvidia/Autoware/ros/nodeshell/LED_detection.sh";
const char *left_cmd = "/home/nvidia/Autoware/ros/nodeshell/LED_left.sh";
const char *right_cmd = "/home/nvidia/Autoware/ros/nodeshell/LED_right.sh";
const char *nostate_cmd = "/home/nvidia/Autoware/ros/nodeshell/LED_nostate.sh";

void monitorstatusCallback(const carctl_msgs::monitor_status &m_status)
{

  std::string service_name = m_status.service_name;
  int status = m_status.status;
  std::string error_msg = m_status.error_msg;

  //エラーリストの検索中にリストを更新されても検索結果に影響を与えないようにリストを複製する
  std::vector<std::string> tmp_error_list;
  copy(error_list.begin(), error_list.end(), back_inserter(tmp_error_list));

  if (service_name == "velocity_set")
  {
    //障害物検知状態を取得
    if (status == ERROR)
    {
      //異常の場合
      is_detection_lidar_ = DETECTION;
    }
    else
    {
      //正常の場合
      is_detection_lidar_ = NO_DETECTION;
    }
  }
  else if (service_name == "plc")
  {
    std::string error_id_str = error_msg.substr(6, 1);
    int errorid = atoi(error_id_str.c_str());

    //２Dセンサー検知状態を取得
    if (status == ERROR)
    {
      switch (errorid)
      {
      case 4: // 2Dライダー（減速）→使用停止し、plc_2_0へ移行
      case 5: // 2Dライダー（停止）
      case 6: // 2Dライダー（エラー）
        //2Dセンサーで物体検知している場合
        is_detection_2d_ = DETECTION;
        break;
      default:
        is_plc_error_ = ERROR;
      }
    }
    else if (status == WARN) {
      if (errorid == 0) { is_detection_2d_ = DETECTION; } // 2Dライダー（減速）
    }
    else
    {
      //正常の場合
      is_detection_2d_ = NO_DETECTION;
      is_plc_error_ = NORMAL;
    }
  }
  else if (status == ERROR)
  {
    //物体検知以外のエラー
    //物体検知以外のエラーも複数同時に発生、解消することがあるため、サービスネーム毎に発生を管理する
    std::vector<std::string>::iterator result = std::find(tmp_error_list.begin(), tmp_error_list.end(), service_name);
    if (result == tmp_error_list.end())
    {
      //エラーリストに存在しない場合退避する
      error_list.push_back(service_name);
    }
  }
  else
  {
    //正常
    //エラーリストの存在する場合削除する
    error_list.erase(std::remove(error_list.begin(), error_list.end(), service_name), error_list.end());
  }

  if (error_list.size() == 0)
  {
    //エラーリストにエラーが存在しない場合、異常状態を解消
    is_error_ = NORMAL;
  }
  else
  {
    //エラーリストにエラーが存在する場合、物体検知以外の異常が発生している
    is_error_ = ERROR;
  }
}

void plcControlCallBack(const udp_msgs::UdpControlPacket &msg)
{
  //ウィンカーを取得
  if (msg.winkerL == 1 && msg.winkerR == 0)
  {
    //左折中
    turn_signal_state_ = LEFT;
  }
  else if (msg.winkerL == 0 && msg.winkerR == 1)
  {
    //右折中
    turn_signal_state_ = RIGHT;
  }
  else
  {
    //直進、その他
    turn_signal_state_ = STRAIGHT;
  }
}

void plcSensorCallBack(const udp_msgs::UdpSensorPacket &msg)
{
  //走行状態を取得
  if (msg.PLC_vel > 0.0)
  {
    //走行中の場合
    is_run_ = RUN;
  }
  else
  {
    //停止中の場合
    is_run_ = STOP;
  }
}

void main_loop_callback(const ros::TimerEvent &)
{
  printf("is_error_:%d, is_plc_error_:%d, is_run_:%d, turn_signal_state_:%d, is_detection_lidar_:%d, is_detection_2d_:%d", is_error_, is_plc_error_, is_run_, turn_signal_state_, is_detection_lidar_, is_detection_2d_);
  printf("\n");

  //表示には優先順位があるため、else ifで判定する
  if (is_error_ == ERROR || is_plc_error_ == ERROR)
  {
    //エラーリストにエラーが存在するかplcで2Dセンサー以外のエラーが発生している場合は異常状態になる
    //異常状態を表示する
    printf("state:error");
    //表示コマンドを実行
    system(error_cmd);
  }
  else if (is_detection_lidar_ == DETECTION || is_detection_2d_ == DETECTION)
  {
    //障害物検知状態を表示する
    printf("state:detection");
    //表示コマンドを実行
    system(detection_cmd);
  }
  else if (is_run_ == STOP)
  {
    //停止状態を表示する
    printf("state:stop");
    //表示コマンドを実行
    system(stop_cmd);
  }
  else if (turn_signal_state_ == LEFT)
  {
    //左折状態を表示する
    printf("state:left");
    //表示コマンドを実行
    system(left_cmd);
  }
  else if (turn_signal_state_ == RIGHT)
  {
    //右折状態を表示する
    printf("state:right");
    //表示コマンドを実行
    system(right_cmd);
  }
  else if (is_run_ == RUN)
  {
    //自動運転状態を表示する
    printf("state:run");
    //表示コマンドを実行
    system(auto_cmd);
  }
  else
  {
    //表示しない
    printf("state:no state");
    //表示コマンドを実行
    system(nostate_cmd);
  }
  printf("\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ctl_led");
  ros::NodeHandle n;

  // sub
  ros::Subscriber sub_led_error_state = n.subscribe("monitor_status", 10, monitorstatusCallback);
  ros::Subscriber sub_plc_con = n.subscribe("plc_control_packet", 10, plcControlCallBack);
  ros::Subscriber sub_plc_sen = n.subscribe("plc_sensor_packet", 10, plcSensorCallBack);

  ros::Timer timer = n.createTimer(ros::Duration(1), main_loop_callback);

  ros::spin();

  return 0;
}
