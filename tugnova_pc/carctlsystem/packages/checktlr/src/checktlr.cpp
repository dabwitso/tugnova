#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <list>
#include "unistd.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/TrafficLight_Result.h"
#include <carctl_msgs/waiting_msg.h>

#include <udp_msgs/UdpSensorPacket.h>

static const autoware_msgs::WaypointCustom init_wpc{};
static const int16_t RED = 0;
static const int16_t BLUE = 1;
static const int16_t OFF = 0;
static const int16_t ON = 1;
static const int16_t UNKNOWN = 9;

static const int16_t NOW = 0;
static const int16_t INIT = 0;

autoware_msgs::WaypointCustom wpc_;         // CallBackするwaypoint
autoware_msgs::WaypointCustom wpcx{};       // 停止箇所のwaypoint
autoware_msgs::WaypointCustom now_wpc;      // 現在車両がいるwaypoint
autoware_msgs::TrafficLight_Result now_tlr; // カメラから検知している最新の信号機ステータス
int16_t now_srvtlr;                         // 上位から検知している最新の信号機ステータス
float PLC_vel;                              // 現在の走行値（タグノバ）
int16_t GSFlg_ = INIT;                      // 発進ボタンの状態

std::vector<int> tlr_results;
ros::Publisher pub_TlrStartFlg;
ros::Publisher pub_TlrStopFlg;
ros::Publisher pub_ctlreduce;

bool reserveInitialize = false; // 状態クリアの予約. 通信途絶時でもクリア可能にするために設けた.

// 繰り返し実行
float HZ = 100;
// 減速フラグ
bool reduceflg = false;
// 一時停止フラグ
bool pauseflg = false;
// 一時停止時間(秒)
int PauseTime = 3;

// 信号結果Callback
void TrafficLightsCallback(const autoware_msgs::TrafficLight_Result &tlr)
{
  now_tlr = tlr;
}

// 上位結果Callback
void CheckSrvTlrCallback(const carctl_msgs::waiting_msg &srvtlr)
{
  std::string waitingFor = srvtlr.waitingFor;
  std::string trafficLightState = srvtlr.trafficLightState;

  // 信号待ちの場合
  if (waitingFor == "trafficLight")
  {
    // 赤信号
    if (trafficLightState == "red")
    {
      now_srvtlr = RED;
    }
    // 青信号
    else if (trafficLightState == "blue")
    {
      now_srvtlr = BLUE;
    }
    // 不明
    else if (trafficLightState == "noSignal")
    {
      now_srvtlr = UNKNOWN;
    }
    // 信号待ちではない箇所（状態不整合）
    else if (trafficLightState == "none")
    {
      now_srvtlr = UNKNOWN;
    }
  }
  // 信号待ちではない箇所
  else
  {
    now_srvtlr = UNKNOWN;
  }
}

// WaypointCallback
void WaypointsCallback(const autoware_msgs::Lane &lane)
{
  wpc_ = lane.waypoints.at(NOW).wpc;
}

// UdpSensorPacketCallback
void PlcsensorCallback(const udp_msgs::UdpSensorPacket &UdpSensorPacket)
{
  // 走行値
  PLC_vel = UdpSensorPacket.PLC_vel;
}

//機台車両ボタンによる強制走行
void GpioStartCallBack(const std_msgs::Int16 &flg)
{
  GSFlg_ = flg.data;
}

// 発進処理
void TlrStartFlg()
{
  std_msgs::Int16 msg;
  msg.data = ON;
  pub_TlrStartFlg.publish(msg);  /* on:1,off:0  */
  wpcx = init_wpc;              /* 停止位置のWaypoint初期化 */
}

// 停止処理
void TlrStopFlg()
{
  // 走行値が出ている間、停止フラグをオンにする
  if (PLC_vel > 0.0)
  {
    std_msgs::Int16 msg;
    msg.data = ON;
    pub_TlrStopFlg.publish(msg); /* on:1,off:0  */
  }
//  else
//  {
//    pub_TlrStopFlg.publish(OFF); /* on:1,off:0  */    
//  }
  
  wpcx = now_wpc;             /* 停止位置のwaypointセット */
}

// 信号機結果確認（カメラ）
int CheckTlrStatus()
{
  // 最新の信号検知結果から配列のデータを確認する
  for (auto itr = tlr_results.begin(); itr != tlr_results.end(); ++itr)
  {
    // 赤か青が存在する場合ステータスを更新してループを抜ける
    printf(" %d,", *itr);
    printf("\n");
    if (*itr == RED or *itr == BLUE)
    {
      // 0=赤, 1=青
      return *itr;
    }
  }
  // 不明の場合赤で返す
  return RED;
}

// 信号機結果更新
void UpdateTlrResult(bool &cameraid)
{
  // 信号検知結果が最大容量より多い場合、配列の最初に追加したデータ（末尾のデータ）を削除
  if (tlr_results.size() > 100)
  {
    tlr_results.pop_back();
  }
  tlr_results.insert(tlr_results.begin(), now_tlr.status);

  if (cameraid)
  {
    // カメラの信号機結果を挿入
    tlr_results.insert(tlr_results.begin(), now_tlr.status);
  }
  else
  {
    // 上位の信号機結果を挿入
    tlr_results.insert(tlr_results.begin(), now_srvtlr);
  }
}

// 信号機の確認
void DiffcheckTlr()
{
  bool cameraid;

  // ルートファイルに信号機IDがセットされてない場合、上位の結果をセットする。
  if (now_wpc.tlr.cameraid == "NULL")
  {
    fprintf(stdout, "DEBUG: DiffcheckTlr: 上位制御 \n");
    cameraid = false;
    UpdateTlrResult(cameraid);
  }
  // ルートファイルに信号機IDがセットされている
  // 信号結果が検出されない場合
  else
  {
    if (now_tlr.id == "")
    {
      fprintf(stdout, "DEBUG: DiffcheckTlr: 対象の信号機なし [%s] \n", now_wpc.tlr.cameraid.c_str());
    }
    // 認識した信号機のIDが一致した場合、信号結果を格納し判定する
    else if (now_wpc.tlr.cameraid == now_tlr.id)
    {
      fprintf(stdout, "DEBUG: DiffcheckTlr: 信号機一致 \n");
      // 信号結果更新
      cameraid = true;
      UpdateTlrResult(cameraid);
    }
    else if (now_wpc.tlr.cameraid != now_tlr.id)
    {
      fprintf(stdout, "DEBUG: DiffcheckTlr: 信号機不一致 \n");
    }
  }
}

void CheckWaypoints()
{
  if (wpcx.waypoint_id == INIT)
  {
    now_wpc = wpc_;
    return;
  }

  // CallBackのWaypointと停止箇所が同じなら停止箇所のWaypointを保持する
  if (wpc_.waypoint_id == wpcx.waypoint_id)
  {
    now_wpc = wpc_;
  }
  // CallBackのWaypointが停止箇所より先に行っていれば停止箇所のWaypointを保持する
  else if (wpc_.waypoint_id > wpcx.waypoint_id)
  {
    // 強制走行がONならばCallBackのWaypointに書き換える
    if (GSFlg_ == ON)
    {
      now_wpc = wpc_;
      wpcx = init_wpc; /* 停止位置のWaypoint初期化 */
    }
    else
    {
      now_wpc = wpcx;
    }
  }
  // 例外
  else if (wpc_.waypoint_id < wpcx.waypoint_id)
  {
    now_wpc = wpc_;
    wpcx = init_wpc; /* 停止位置のWaypoint初期化 */
  }
}

// 停止制御
void ctlrun(int &runflg)
{
  //一時停止フラグ判定
  if (pauseflg)
  {
    fprintf(stdout, "DEBUG: ctlrun: 一時停止[%d秒] \n", PauseTime);
    TlrStopFlg();
    // スリープ
    sleep(PauseTime);
    pauseflg = false;
  }

  if (runflg == BLUE)
  {
    // 走行許可
    fprintf(stdout, "DEBUG: ctlrun: 走行許可 \n");
    TlrStartFlg();
    // 信号機結果列クリア
    tlr_results.clear();
  }
  else if (runflg == RED)
  {
    // 走行停止
    fprintf(stdout, "DEBUG: ctlrun: 走行停止 \n");
    TlrStopFlg();
  }
  // 例外
  else
  {
    // 走行停止
    fprintf(stdout, "DEBUG: ctlrun: 例外により走行停止 \n");
    TlrStopFlg();
  }
}

void checktlr()
{
  std_msgs::Int16 msg;
  // 停止箇所かCallbackしたWaypointをnow_wpcにセットする
  CheckWaypoints();

  // 信号検知範囲外の場合、走行継続する
  if (now_wpc.tlr.range == 0 || GSFlg_ == ON)
  {
    fprintf(stdout, "DEBUG: ctlrun: 信号機判定範囲外のため走行継続 \n");

    if (reserveInitialize) {
      now_srvtlr = UNKNOWN;
      tlr_results.clear();
      reserveInitialize = false;
    }

    // 減速なし
    msg.data = BLUE;
    pub_ctlreduce.publish(msg);
    // 停止なし
    TlrStartFlg();
    return;
  }

  // 信号機の判定と状態取得
  DiffcheckTlr();
  reserveInitialize = true; // 信号機判定エリアを抜けた後、checksrvから状態更新が来なくとも、一度信号機状態をクリアしたいので、初期化の予約を行う

  /* 現在位置の判定
  tlr_camera.range
  1:通過確認
  2:停止行動（一時停止なし）
  3:停止行動（一時停止）
  4:停止位置
  */
  switch (now_wpc.tlr.range)
  {
  // 通過確認範囲は信号機確認し停止判断を行う
  case 1:
  {
    fprintf(stdout, "DEBUG: checktlr: 通過確認 \n");
    // 減速なし
    msg.data = BLUE;
    pub_ctlreduce.publish(msg);
    break;
  }
  // 停止行動範囲（一時停止なし）は赤の場合、停止位置まで減速する、青の場合、減速せず通過する
  case 2:
  {
    fprintf(stdout, "DEBUG: checktlr: 停止行動（一時停止なし） \n");
    //一時停止フラグOFF
    pauseflg = false;
    // 信号機の状態判定
    msg.data = CheckTlrStatus();
    // 減速判定
    pub_ctlreduce.publish(msg);
    break;
  }
  // 停止行動範囲（一時停止あり）は停止位置まで減速する
  case 3:
  {
    fprintf(stdout, "DEBUG: checktlr: 停止行動（一時停止あり） \n");
    //一時停止フラグON
    pauseflg = true;
    // 現在の信号の状態取得
    msg.data = CheckTlrStatus();
    // 減速実施
    pub_ctlreduce.publish(msg);
    break;
  }
  // 停止位置範囲は赤の場合、停止する、青の場合、通過する
  case 4:
  {
    fprintf(stdout, "DEBUG: checktlr: 停止位置 \n");
    // 信号機の状態判定
    int flg = CheckTlrStatus();
    msg.data = flg;
    // 減速判定
    pub_ctlreduce.publish(msg);
    // 停止実施判定
    ctlrun(flg);

    break;
  }
  // 不正な値の場合停止する。
  default:
  {
    fprintf(stdout, "DEBUG: checktlr: 不正な値のため停止 \n");
    // 停止
    TlrStopFlg();
    break;
  }
  }

  // Debug
  printf("wpcx.WpId:%d", wpcx.waypoint_id);
  printf(" now_wpc.WpId:%d", now_wpc.waypoint_id);
  printf(" wTlrRange:%d", now_wpc.tlr.range);
  printf(" wTlrId:%s", now_wpc.tlr.cameraid.c_str());
  printf(" tTlrId:%s", now_tlr.id.c_str());
  printf(" TlrStatus:%d", now_tlr.status);
  printf("\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "checktlr");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("PauseTime", PauseTime);
  ROS_DEBUG("PauseTime[%d]", PauseTime);

  ros::Subscriber sub_tlr = n.subscribe("TrafficLight_Result", 10, TrafficLightsCallback);
  ros::Subscriber sub_chksrv = n.subscribe("WaitingFor_Result", 10, CheckSrvTlrCallback);
  ros::Subscriber sub_waypoints = n.subscribe("safety_waypoints", 10, WaypointsCallback);
  ros::Subscriber sub_plcsensor = n.subscribe("plc_sensor_packet", 10, PlcsensorCallback);
  pub_TlrStartFlg = n.advertise<std_msgs::Int16>("TlrStartFlg", 10, true);
  pub_TlrStopFlg = n.advertise<std_msgs::Int16>("TlrStopFlg", 10, true);
  pub_ctlreduce = n.advertise<std_msgs::Int16>("TlrReduceFlg", 10, true);
  //車両機台スイッチからの走行指示（強制走行）
  ros::Subscriber sub_GpioStart = n.subscribe("GpioStartFlg", 10, GpioStartCallBack);
  ros::Rate loop_rate(HZ);
  while (ros::ok())
  {
    checktlr();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
