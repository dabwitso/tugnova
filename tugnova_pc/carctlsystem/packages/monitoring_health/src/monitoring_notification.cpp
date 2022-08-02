#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <map>
#include <string>
#include <vector>

#include <udp_msgs/UdpSensorPacket.h>
#include <carctl_msgs/monitor_status.h>
#include <carctl_msgs/phone_msg.h>
#include <carctl_msgs/emergency_status.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pub_phoneMsg;
ros::Publisher pub_msChecker;
ros::Publisher pub_plcmsg;
ros::Publisher pub_emergency_error;

static const int16_t INFO = 0;
static const int16_t ERROR = 1;
static const int16_t WARN = 2;
static const int16_t STOP = 0;
static const int16_t RUN = 1;
static const int16_t NOT_OCCURRED = 1;
static const int16_t OCCURRED = 0;

std::string selfLocalization;
std::string allresult_messageId;
std::map<std::string, int> mlist;

bool all_result = false;
int wait_state = 0;
float plc_val = 0.0;
double notification_time_threshold = 1.0;
double notification_time = 0.0;

//initialposeで復帰できる異常
int16_t cancel_possible_emergency_error_by_initialpose = NOT_OCCURRED;
//自己位置復帰で復帰できる異常
int16_t cancel_possible_emergency_error_by_scan = NOT_OCCURRED;
//自己位置復帰で復帰できない異常
int16_t cancel_impossible_emergency_error_by_scan = NOT_OCCURRED;

std::vector<std::string> scan_respawn_node_list;

/*
  service_name: 
    - check_cpu          // CPU監視
    - check_mem          // メモリ監視
    - check_proc         // プロセス監視
    - ping_device        // デバイス監視
    - ping_node          // ノード監視
    - lost_localization  // 迷子/脱線検知
    - check_change_route // ルート変更
    - velocity_set       // 物体検知
    - server             // 停止箇所、上位通信
*/

// 現在時刻をミリ秒で取得する
double calc_time()
{
  struct ::timespec getTime;
  clock_gettime(CLOCK_MONOTONIC, &getTime);
  return (getTime.tv_sec + getTime.tv_nsec * 1e-9) * 1000;
}

// 障害通知の要否を返す
bool maplistNotification(const carctl_msgs::monitor_status &m_status)
{

  bool notification_flg = false;
  int mlist_check;
  std::string service_name = m_status.service_name;
  std::string error_msg_ = m_status.error_msg;
  int status_ = m_status.status;

  // 経路作成モード時のメッセージは通知する
  if (service_name == "waypoint_editor")
  {
    notification_flg = true;
  }

  // 異常/警告検知しているならば通知(ture)する
  if (status_ == ERROR || status_ == WARN)
  {
    fprintf(stdout, "DEBUG: maplistNotification: 異常/警告検知通知\n");
    notification_flg = true;
  }

  // 上位通信正常通知
  if (error_msg_ == "server_0_2")
  {
    fprintf(stdout, "DEBUG: maplistNotification: 上位通信正常通知\n");
    notification_flg = true;
  }

  fflush(stdout);
  return notification_flg;
}

// 障害通知の配列を作成・ステータス格納
void maplistUpdate(const carctl_msgs::monitor_status &m_status)
{
  std::string service_name = m_status.service_name;
  int status = m_status.status;

  // ステータスを配列に格納する
  mlist[service_name] = status;
}

// 自己位置の確認
std::string setLocalization()
{
  std::string selfLocalization_ = "init";
  int status_;

  // 配列がない場合
  if (mlist.empty())
  {
    fprintf(stdout, "DEBUG: setLocalization: 自己位置状態なし\n");
    return selfLocalization_;
  }

  // 通知があるか確認
  if (mlist.count("lost_localization"))
  {
    // 通知のステータスを格納
    status_ = mlist["lost_localization"];

    // 自己位置の状態を返す
    if (status_ == INFO)
    {
      fprintf(stdout, "DEBUG: setLocalization: 自己位置正常状態\n");
      selfLocalization_ = "acquired";
    }
    else if (status_ == WARN)
    {
      fprintf(stdout, "DEBUG: setLocalization: 自己位置警告状態\n");
      selfLocalization_ = "warning";
    }
    else if (status_ == ERROR)
    {
      fprintf(stdout, "DEBUG: setLocalization: 自己位置異常状態\n");
      selfLocalization_ = "lost";
    }
  }

  return selfLocalization_;
}

// 障害通知が全て正常か確認
int maplistCheck()
{
  std::map<std::string, int>::iterator it;
  std::map<std::string, int>::iterator end;
  int result = INFO;
  std::string key;
  int velue;

  // 配列に格納されている障害通知のステータスを確認する
  for (it = mlist.begin(), end = mlist.end(); it != end; ++it)
  {
    // int selfLocalization;
    key = it->first;
    velue = it->second;

    // 異常検知が一つでもあれば、falseをセットする
    if (velue == ERROR)
    {
      fprintf(stdout, "DEBUG: maplistCheck: [%s] : 異常検知中\n", key.c_str());
      result = ERROR;
      break;
    }
    else if (velue == WARN && key != "server") // server(=上位通信)の警告は画面上ランプ点灯のみでメッセージは表示しないため、特例で正常稼働に落とす
    {
      fprintf(stdout, "DEBUG: maplistCheck: [%s] : 警告稼働中\n", key.c_str());
      result = WARN;
    }
    else
    {
      fprintf(stdout, "DEBUG: maplistCheck: [%s] : 正常稼働中\n", key.c_str());
    }
  }

  fflush(stdout);
  return result;
}


// 端末通知機能へメッセージの通知
void wholeNotification(const int &all_result_, const int16_t &wait_state_, const float &plc_val)
{
  carctl_msgs::phone_msg p_msgs;
  p_msgs.selfLocalization = selfLocalization;
  // fprintf(stdout, "DEBUG: wholeNotification: service_name=%s,error_msg=%s\n", service_name_.c_str(), error_msg_.c_str());

  if (all_result_ == WARN || all_result_ == ERROR)
  {
    // 異常がある場合通知しない
    return;
  }
  // 全サービスが正常稼働中の場合（警告を含む場合アリ）
  //正常を通知する
  fprintf(stdout, "DEBUG: wholeNotification: 正常稼働の通知実行\n");
  switch (wait_state_)
  {
  case 0:
    if (plc_val == 0.0)
    {
      p_msgs.messageId = "status_0_0";
    }
    else
    {
      p_msgs.messageId = "status_0_1";
    }
    break;
  case 1:
    // 上位待ち
    p_msgs.messageId = "server_0_0";
    break;
  case 2:
    // 発進ボタン待ち
    p_msgs.messageId = "server_0_1";
    break;
  case 3:
    // 信号待ち
    p_msgs.messageId = "traffic_signal_0_0";
    break;
  case 5:
    // 一旦停止待ち
    p_msgs.messageId = "velocity_set_0_1";
    break;
  default:
    p_msgs.messageId = "status_0_0";
    break;
  }

  if (allresult_messageId != p_msgs.messageId)
  {
    notification_time = 0.0;
    pub_phoneMsg.publish(p_msgs);
  }
  else
  {
    if (notification_time == 0.0)
    {
      notification_time = calc_time();
    }
    double diff_time = calc_time() - notification_time;
    if (diff_time > notification_time_threshold)
    {
      notification_time = 0.0;
      pub_phoneMsg.publish(p_msgs);
    }
  }
  allresult_messageId = p_msgs.messageId;
  fflush(stdout);
}

// 端末通知機能へ個別メッセージの通知
void individualNotification(const carctl_msgs::monitor_status &m_status)
{
  carctl_msgs::phone_msg p_msgs;
  p_msgs.messageId = m_status.error_msg;
  p_msgs.selfLocalization = selfLocalization;
  std::string service_name_ = m_status.service_name;

  fprintf(stdout, "DEBUG: individualNotification: [%s] : 個別サービスの通知\n", service_name_.c_str());
  pub_phoneMsg.publish(p_msgs);
  fflush(stdout);
}

// 異常がある場合、車両停止する
void stopCarcontrol(const carctl_msgs::monitor_status &m_status)
{
  std::string service_name_ = m_status.service_name;
  std::string error_msg_;
  int errorid;

  if (service_name_ == "velocity_set")
  {
    fprintf(stdout, "DEBUG: stopCarcontrol: 障害物検知のため制御不要\n");
    return;
  }

  fprintf(stdout, "DEBUG: stopCarcontrol: 走行停止状態\n");
  // 停車両停止
  std_msgs::Int16 msg;
  msg.data = STOP;
  pub_msChecker.publish(msg);

  fflush(stdout);
}

// 全てのステータスが正常な場合、端末による走行再開を許可する
void startCarcontrol()
{
  fprintf(stdout, "DEBUG: startCarcontrol: 走行可能状態\n");
  std_msgs::Int16 msg;
  msg.data = RUN;
  pub_msChecker.publish(msg);

  fflush(stdout);
}

void setOccurred(const carctl_msgs::monitor_status &m_status)
{
  if (m_status.error_msg == "lost_localization_1_0"
  || m_status.error_msg == "lost_localization_1_1"
  || m_status.error_msg == "lost_localization_1_2")
  {
    //自己位置異常、脱線が発生した場合
    //initialposeでの解除可能な異常発生
    cancel_possible_emergency_error_by_initialpose = OCCURRED;
  }
  else
  {
    
    std::vector<std::string>::iterator result = std::find(scan_respawn_node_list.begin(), scan_respawn_node_list.end(), m_status.error_msg);
    if (result == scan_respawn_node_list.end())
    {
      //「自己位置復帰で復帰するノードの停止」以外の異常が発生した場合
      //スキャンでの解除不可能状態な異常発生
      cancel_impossible_emergency_error_by_scan = OCCURRED;
    }
    else
    {
      //自己位置復帰で復帰するノードが停止した場合
      //スキャンでの解除可能な異常発生
      cancel_possible_emergency_error_by_scan = OCCURRED;
    }
  }
}

// PLCへの通知
void plcNotification(int all_result_, const carctl_msgs::monitor_status &m_status)
{
  std_msgs::Int16 msg;
  carctl_msgs::emergency_status emergency_status;

  emergency_status.service_name = "monitoring_notification";

  if (all_result_ != ERROR)
  {
    fprintf(stdout, "DEBUG: plcNotification: PLCへ正常状態を通知\n");
    msg.data = INFO;
    emergency_status.status = INFO;
  }
  else
  {
    fprintf(stdout, "DEBUG: plcNotification: PLCへエラー状態を通知\n");
    msg.data = ERROR;
    emergency_status.status = ERROR;

    if (m_status.status != ERROR)
    {
      //正常、警告通知が来た場合には正常を通知する
      emergency_status.status = INFO;
    }
    else
    {
      //異常通知の場合
      if (m_status.service_name == "velocity_set" || m_status.service_name == "plc")
      {
        //物体検知時またはplc異常時には緊急停止通知を正常で通知する
        emergency_status.status = INFO;
      }
      else
      {
        //緊急停止が必要な異常が発生した場合
        //異常によって異常発生状態を設定
        setOccurred(m_status);
      }
    }
  }

  //緊急停止が必要な異常が発生した場合、緊急停止状態を保持する
  if (cancel_possible_emergency_error_by_initialpose == OCCURRED)
  {
    emergency_status.status = ERROR;
  }
  if (cancel_possible_emergency_error_by_scan == OCCURRED)
  {
    emergency_status.status = ERROR;
  }
  if (cancel_impossible_emergency_error_by_scan == OCCURRED)
  {
    emergency_status.status = ERROR;
  }

  pub_plcmsg.publish(msg);
  pub_emergency_error.publish(emergency_status);

  fflush(stdout);
}

// メイン処理
void monitoringNotification(const carctl_msgs::monitor_status &m_status)
{
  int all_result_; // 全正常: INFO、いずれかが異常: ERROR, 異常がない状況で警告がある: WARN

  // ROS_DEBUG("Infomation: [%s] \n", m_status.error_msg.c_str());
  // 異常通知の場合、車両停止
  if (m_status.status == ERROR)
  {
    stopCarcontrol(m_status);
  }

  // 自己位置の状態取得
  selfLocalization = setLocalization();

  // ステータスの通知要否判定
  // 個別メッセージの端末通知機能への連携
  if (maplistNotification(m_status))
  {
    individualNotification(m_status);
  }

  // 配列へステータスの格納
  maplistUpdate(m_status);

  // 配列の全ステータス判定
  all_result_ = maplistCheck();

  // 全てのサービスが正常もしくは警告な場合、車両再発進許可
  if (all_result_ == INFO || all_result_ == WARN) { startCarcontrol(); }

  // 車両状態の端末通知機能への連携（警告時は警告用のメッセージが既に流れているため、正常時メッセージは流さない。流すと両方表示されてしまうため。）
  wholeNotification(all_result_, wait_state, plc_val);

  // PLCへの通知
  plcNotification(all_result_, m_status);
}

// CallBack
void monitorstatusCallback(const carctl_msgs::monitor_status &m_status)
{
  carctl_msgs::monitor_status msg;
  msg = m_status;

  monitoringNotification(msg);
}
// CallBack
void waitstateCallback(const std_msgs::Int16 &state)
{
  wait_state = state.data;
}
// CallBack
void PlcsensorCallback(const udp_msgs::UdpSensorPacket &UdpSensorPacket)
{
  plc_val = UdpSensorPacket.PLC_vel;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose)
{
  cancel_possible_emergency_error_by_initialpose = NOT_OCCURRED;
}

void scanRespawnResultCallback(const std_msgs::Int16 msg)
{
  cancel_possible_emergency_error_by_scan = msg.data;
}


int main(int argc, char **argv)
{
  fprintf(stdout, "DEBUG: monitoring_notification start.\n");
  
  scan_respawn_node_list.push_back("node_1_6");
  scan_respawn_node_list.push_back("node_1_33");
  scan_respawn_node_list.push_back("node_1_53");
  scan_respawn_node_list.push_back("node_1_52");
  scan_respawn_node_list.push_back("plc_converter_1_0");
  
  // ros 初期化
  ros::init(argc, argv, "monitoring_notification");
  ros::NodeHandle n;
  // pub
  pub_phoneMsg = n.advertise<carctl_msgs::phone_msg>("phone_msg", 10); // 端末通知機能
  pub_msChecker = n.advertise<std_msgs::Int16>("MsCheckFlg", 10);      // 走行制御統合機能（障害通知FLG）
  pub_plcmsg = n.advertise<std_msgs::Int16>("plc_errormsg", 10);       // COMSドライバ
  pub_emergency_error = n.advertise<carctl_msgs::emergency_status>("/emergency_error", 10);

  // sub
  ros::Subscriber sub_monitor_status = n.subscribe("monitor_status", 10, monitorstatusCallback);
  ros::Subscriber sub_waitstate_flg = n.subscribe("waitstate_flg", 10, waitstateCallback);
  ros::Subscriber sub_plcsensor = n.subscribe("plc_sensor_packet", 10, PlcsensorCallback);
  ros::Subscriber sub_initpose = n.subscribe("/initialpose", 10, initialPoseCallback);
  ros::Subscriber sub_scan_respawn_result = n.subscribe("/scan_respawn_result", 10, scanRespawnResultCallback);
  
  ros::spin();

  fprintf(stdout, "DEBUG: monitoring_notification end.\n");

  return 0;
}
