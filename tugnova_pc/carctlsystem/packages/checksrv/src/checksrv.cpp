#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <pthread.h>
#include <cstdlib>
#include <curl/curl.h>

#include "get_ip_address_string.h"
#include <safety_waypoints_callback.h>
#include "post_vms_control.h"
#include "plc_sensor_packet_callback.h"
#include <carctl_msgs/waiting_msg.h>
#include <carctl_msgs/monitor_status.h>
#include <carctl_msgs/battery_status.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <util.h>

int initial_transmission_flag = 0;
// パラメータ初期値
std::string post_url = "http://192.168.1.200:5000/vms/control";
int post_timeout = 1;
int post_force_timeout = 10;
std::string car_vehicleId = "1";
char str_ip_addr[256];
// std::string car_IPaddr = "192.168.1.10";
std::string car_Iface = "wlp59s0";
std::string c_route_id = "GASTANK_0";
std::string error_id = "status_0_0";
std::string battery_scale = "0";
std::string battery_color = "0";

double process_rate = 0.5;
int process_wpoint_threshold = 10;
double process_loop_timer = 0.01;

//ros::Publisher pub_checksrv;
ros::Publisher pub_SrvStartFlg;
ros::Publisher pub_SrvStopFlg;
ros::Publisher pub_ConnectStartFlg;
ros::Publisher pub_ConnectStopFlg;
ros::Publisher pub_checkpost;
ros::Publisher pub_routechange;
ros::Publisher pub_waitingfor;
ros::Publisher pub_statecmd;
ros::Publisher pub_checksrv_waypointid;
pthread_mutex_t mymutex = PTHREAD_MUTEX_INITIALIZER;

//統合機能からの上位サーバ走行判断許可
static const int16_t OFF = 0;
static const int16_t ON = 1;
static const int16_t STOP = 0;
static const int16_t RUN = 1;
static const int16_t INIT = 9;

int16_t CVFlg_ = INIT;
int16_t GSFlg_ = INIT;

int g_arg = 0;
//上位サーバーとの通信状態
// server_connect_state_ = 

double disconnect_time_threshold = 60.0;

//上位サーバーと切断した時間（ミリ秒）
double server_disconnect_time = 0.0;

// 時間取得（ミリ秒）
double last_send_time = 0.0;

//異常状態
const int16_t NOMAL = 0;
const int16_t ERROR = 1;
const int16_t WARN = 2;

const std::string SERVER_SERVICE_NAME = "server";
const std::string SERVER_DISCONNECTING_MSG = "server_2_0";
const std::string SERVER_CONNECTING_MSG = "server_0_2";

const std::string CONNECTION_SERVICE_NAME = "connection";
const std::string SERVER_CONNECTION_CHECK_NOMAL_MSG = "connection_0_0";
const std::string SERVER_CONNECTION_CHECK_WARN_MSG = "connection_2_0";

bool has_remote_stop_warning_message = false;
bool is_stop = false;
bool is_force_stop = false;
const std::string REMOTE_NAME = "remote_stop";
const std::string REMOTE_STOP_MSG = "remote_stop_2_0";
const std::string REMOTE_FORCE_STOP_MSG = "remote_stop_2_1";


bool onWaitingPoint = false;

int test_drive = 0;

//上位通信確認地点から通信失敗している場合に車両を停止させる秒数
double post_disconnect_decision = 0.0;

bool is_post_disconnect = false;

//異常検知機能に異常を通知
void publishMonitorStatus(std::string service_name, int16_t status, std::string error_msg)
{
  carctl_msgs::monitor_status msg;
  //ノード名は固定
  msg.service_name = service_name;
  //状態を格納
  msg.status = status;
  //エラーメッセージを格納
  msg.error_msg = error_msg;
  
  //パブリッシュ
  pub_statecmd.publish(msg);

  ros::spinOnce();
}


// 経過時間チェック
//  走行時 POST 実行の間隔を調整する
//  最後に送信した時間から指定の時間経過していた場合に送信対象とする
//  i : チェック用経過時間間隔（秒）
//  o : false:未経過 true:既経過
bool check_elapsed_time(float target)
{
  return (((target * 1000.0) <= (calc_time() - last_send_time)) ? true : false);
}

// 発進処理
void SrvStartFlg()
{
  onWaitingPoint = false;

  std_msgs::Int16 result;
  result.data = ON;
  pub_SrvStartFlg.publish(result);  /* on:1,off:0  */
}

// 停止処理
void SrvStopFlg()
{
  onWaitingPoint = true;

  // 走行値が出ている間かつ走行ボタンがOFFの間、停止フラグをオンにする
  std_msgs::Int16 result;
  result.data = ON;
  pub_SrvStopFlg.publish(result); /* on:1,off:0  */
}

// 上位通信途絶時の停止指示を出力する
void publishConnectStartFlg()
{
  std_msgs::Int16 result;
  result.data = ON;
  pub_ConnectStartFlg.publish(result); /* on:1,off:0  */
}

// 上位通信途絶時の停止指示を出力する
void publishConnectStopFlg()
{
  std_msgs::Int16 result;
  result.data = ON;
  pub_ConnectStopFlg.publish(result); /* on:1,off:0  */
}

void connection_check(int post_result) {
  double connection_check_time = getConnectionCheckPointTime();
  ROS_WARN("connection_check_time[%If], post_result[%d], post_disconnect_decision[%If]", connection_check_time, post_result, post_disconnect_decision);
  if (post_result == -1)
  {
    //上位との接続が失敗している場合
    if (connection_check_time != 0.0)
    {
      //上位通信確認地点に到達している(確認地点到達時間が初期値ではない)場合
      
      double diff_time = calc_time() - connection_check_time;
      if ((post_disconnect_decision * 1000.0) <= diff_time)
      {
        //上位通信確認地点到達時刻から閾値時間、通信が失敗している場合車両を停止
        publishConnectStopFlg();
        //警告を出力する
        is_post_disconnect = true;
        setConnectionCheckPointTime(0.0);
      }
    }
  }
  else
  {
    //上位との接続が成功している場合、上位通信確認地点到達時刻を初期化
    setConnectionCheckPointTime(0.0);
    publishConnectStartFlg();
    is_post_disconnect = false;
  }
}

// POST共通化
//  o : json document
int request_execute(rapidjson::Document &d_)
{
  if (test_drive == 1) {
    // fprintf(stdout, "DEBUG: 試走取り込み中は上位送信は行わない\n");
    return -3;
  }

  std::vector<int> waypoint_ids = getTransitRecordList();
  if (waypoint_ids.empty()) {
    // fprintf(stdout, "DEBUG: WaypointIDリストが空のため次回上位送信は行なわれない\n");
    return -2;
  }

  struct _post_param_struct p;

  // PLC からの情報
  int plc_status_ = 0;                    // int16 ECU_vel
  int plc_error_id_ = 0;                  // int16 BrakePotVol
  float plc_average_vehicle_speed_ = 0.0; // float32 PLC_vel
  getPlcSensorInfo(plc_status_, plc_error_id_, plc_average_vehicle_speed_);

  //MAPファイル、ルートファイルの読み込み
  ros::NodeHandle n;
  n.getParam("current_lane_id", c_route_id);

  std::string plc_run_status_ = "idling";
  if (plc_average_vehicle_speed_ > 0.0)
  {
    plc_run_status_ = "running";
  }

  // パラメータ構築
  p.v_id = car_vehicleId;        // 車両ID
  p.ipaddr = str_ip_addr;        // IPアドレス
  p.coms_st = plc_run_status_;       // 車両状態
  p.route_id = c_route_id;       //ルートID
  p.route_last_mod = 0;
  p.waypoint_ids = waypoint_ids;    // waipoint id list 
  p.error_id = error_id;      // エラーコード
  p.battery_scale = battery_scale;
  p.battery_color = battery_color;

  int post_result = send_post_vms_control(p, d_, post_timeout, post_force_timeout);

  if (!is_force_stop) {
    connection_check(post_result);
  } else {
    // 走行時、上位からのその場停止指示がある場合は通信の状態に関わらず強制的に停止させる
    publishConnectStopFlg();
    setConnectionCheckPointTime(0.0);
  }

  //通信失敗（結果がマイナス）の場合は-1を返す
  if (post_result < 0){post_result = -1;}

  return post_result;
}

void publish_topics(std::string& request, std::string& waitingFor, std::string& trafficLightState)
{
  std_msgs::Int8 result;

  if (request == "sync-routes")
  {
    // 地図・経路ファイルの更新あり
    // fprintf(stdout, "DEBUG: publish_topics: 地図・経路ファイルの更新あり request [%s]\n", request.c_str());
    result.data = 1;
    pub_routechange.publish(result);
  }
  else
  {
    // 地図・経路ファイルの更新なし
    result.data = 0;
    pub_routechange.publish(result);
  }

  carctl_msgs::waiting_msg waiting_msg;
  waiting_msg.waitingFor = waitingFor;
  waiting_msg.trafficLightState = trafficLightState;

  // fprintf(stdout, "DEBUG: publish_topics: waitingFor=[%s] trafficLightState=[%s]\n", waitingFor.c_str(), trafficLightState.c_str());
  pub_waitingfor.publish(waiting_msg);
}

/**
 * 上位サーバとの接続が切れた時に実行し、一定時間経過後に通信エラーを行う.
 */
void timerServerDisconnection(int ret, std::string message, int waypoint_id) {
  if (ret != -1) { return; }

  // サーバーとの通信が失敗した場合、切断時間を記録する
  if (server_disconnect_time == 0.0)
  {
    //送信失敗が連続した場合に上書きされないように初回の失敗だけ時間を記録する
    server_disconnect_time = calc_time();
  }

  ROS_WARN("Server Disconnect: %s waypoint[%d]", message.c_str(), waypoint_id);
}

/** 上位からの遠隔停止指示のメッセージ送信. */
void publishRemoteStopMessage() {
  if (is_stop || is_force_stop) {
    has_remote_stop_warning_message = true;
    publishMonitorStatus(REMOTE_NAME, WARN, is_force_stop ? REMOTE_FORCE_STOP_MSG : REMOTE_STOP_MSG);
  } else {
    if (has_remote_stop_warning_message) {
      has_remote_stop_warning_message = false;
      publishMonitorStatus(REMOTE_NAME, NOMAL, ""); // 警告メッセージ表示解除を行う
    }
  }
}


// 発信可能かどうかを判断するcallback処理
// 上位からの情報が発進となるまでループがかかる
//  i : 停止ポイントの情報(waypoint_id)
void check_post_callback(const carctl_msgs::RouteCheck &cb_param_)
{

  ROS_WARN("check_post_callback: WaypointId=[%d] RouteCheckFlg=[%d]", cb_param_.WaypointId, cb_param_.RouteCheckFlg);

  carctl_msgs::RouteCheck r_flg_;
  getRouteCheckInfo(r_flg_);

  ROS_WARN("check_post_callback: cb_param[%d] r_flg_[%d]", cb_param_.WaypointId, r_flg_.WaypointId);
  if (r_flg_.RouteCheckFlg != 0 || cb_param_.WaypointId > r_flg_.WaypointId)
  {
    ROS_WARN("check_post_callback: waiting point loop stop waypoint[%d]", cb_param_.WaypointId);
    onWaitingPoint = false;
    return;
  }

  // 機台車両ボタンによる強制走行
  if (GSFlg_ == ON)
  {
    ROS_WARN("GSFlg_ == ON");
    GSFlg_ = OFF;
    SrvStartFlg();
    passWaitingWaypoint(cb_param_.WaypointId);
    return;
  }

  forcedPassWaitingWaypoint(process_wpoint_threshold);

  rapidjson::Document doc;
  pthread_mutex_lock(&mymutex);
  // fprintf(stdout, "DEBUG: check_post_callback: ret取得前\n");
  int ret = request_execute(doc);
  ROS_WARN("check_post_callback: ret=[%d] CVFlg=[%d]", ret, CVFlg_);
  pthread_mutex_unlock(&mymutex);

  // 上位問い合わせに問題なし
  if (ret == 0)
  {
    std::string request = doc["request"].GetString();
    bool permission = false;
    std::string waitingFor = "none";
    std::string trafficLightState = "none";
    getResultOfWaypoint(doc["results"], cb_param_.WaypointId, permission, waitingFor, trafficLightState, is_stop, is_force_stop);

    //サーバーとの通信が成功した場合、切断時間を初期化する
    server_disconnect_time = 0.0;
    //上位以外の発進許可がある場合、上位問い合わせの結果（permission）を判定
    //機台車両ボタンを押下を検知したら強制的に走行する
    if (CVFlg_ == RUN)
    {
      ROS_WARN("DEBUG: check_post_callback: permission [%s]", (permission ? "true" : "false"));

      // 上位発信許可を判定する
      if (permission && !is_stop && !is_force_stop)
      {
        // 停止ポイント状況確認ループ停止
        // 発進許可あり
        // fprintf(stdout, "DEBUG: check_post_callback: 発進許可あり 停止ポイント状況確認ループ停止 waypoint[%d]\n", cb_param_.WaypointId);

        passWaitingWaypoint(cb_param_.WaypointId);
        clearTransitRecord();

        SrvStartFlg();
      }
      else
      {
        // 停止ポイント状況確認ループ開始（再度）
        // 発進許可なしの時は上位からの停止を継続させ再度問い合わせ
        // fprintf(stdout, "DEBUG: check_post_callback: 停止ポイント状況確認ループ開始（再度） waypoint[%d]\n", cb_param_.WaypointId);

        clearTransitRecordOtherExceptionWaypointID(cb_param_.WaypointId);

        ros::Duration(process_rate).sleep();
        ROS_WARN("publish 1");
        pub_checkpost.publish(cb_param_);
      }
    }
    else
    {
      // 停止ポイント状況確認ループ開始（再度）
      // 発進許可なしの時は上位からの停止を継続させ再度問い合わせ
      // fprintf(stdout, "DEBUG: check_post_callback: 停止ポイント状況確認ループ開始（CVFlg） waypoint[%d]\n", cb_param_.WaypointId);

      ros::Duration(process_rate).sleep();
      ROS_WARN("publish 2");
      pub_checkpost.publish(cb_param_);
    }

    publish_topics(request, waitingFor, trafficLightState);
  }
  else
  {
    timerServerDisconnection(ret, "check_post_callback: Failed to request_execute. stop published.", cb_param_.WaypointId);

    // memo:通信状態不安定を想定してリトライを実行する方針
    // 停止ポイント状況確認ループ開始（再度）
    // 発進許可なしの時は上位からの停止を継続させ再度問い合わせ
    // fprintf(stdout, "DEBUG: check_post_callback: 停止ポイント状況確認ループ開始（再度） waypoint[%d]\n", cb_param_.WaypointId);

    ros::Duration(process_rate).sleep();
    ROS_WARN("publish 3");
    pub_checkpost.publish(cb_param_);
  }

  publishRemoteStopMessage();
}

void *check_route_change_thread(void *param)
{
  int waypoint_id = g_arg;

  rapidjson::Document doc;
  pthread_mutex_lock(&mymutex);
  ROS_WARN("check_route_change_thread: wid=[%d]", waypoint_id);
  int ret = request_execute(doc);
  // // fprintf(stdout, "DEBUG: check_route_change_thread: ret=[%d] \n", ret);
  pthread_mutex_unlock(&mymutex);

  if (ret == 0)
  {
    std::string request = doc["request"].GetString();
    bool permission = false;
    std::string waitingFor = "none";
    std::string trafficLightState = "none";
    getResultOfWaypoint(doc["results"], waypoint_id, permission, waitingFor, trafficLightState, is_stop, is_force_stop);

    clearTransitRecord();

    //サーバーとの通信が成功した場合、切断時間を初期化する
    server_disconnect_time = 0.0;
    publish_topics(request, waitingFor, trafficLightState);
  }
  else
  {
    timerServerDisconnection(ret, "check_route_change_thread: 走行ポイント 上位サーバ送信エラー", waypoint_id);
  }

  publishRemoteStopMessage();
}

//上位連携のエラー状態を確認してメッセージを出力する
void check_connection()
{
  if (is_post_disconnect)
  {
    //通信途絶状態の場合エラーメッセージを出力する
    publishMonitorStatus(CONNECTION_SERVICE_NAME, WARN, SERVER_CONNECTION_CHECK_WARN_MSG);
    //同時に正常メッセージ(SERVER_CONNECTING_MSG)を出力しないようにreturnする
    return;
  }
  else
  {
    publishMonitorStatus(CONNECTION_SERVICE_NAME, NOMAL, SERVER_CONNECTION_CHECK_NOMAL_MSG);
  }

  //サーバーと通信失敗している場合、通信失敗時間を確認し、不通状態であればワーニングを通知する。
  //printf("server_disconnect_time:%if", server_disconnect_time);
  if (server_disconnect_time != 0.0)
  {
    //通信失敗時刻から現在時刻までの差分
    double diff_time = calc_time() - server_disconnect_time;
    if ((disconnect_time_threshold * 1000.0) <= diff_time)
    {
      //通信失敗時間が設定値以上なのでワーニングを発生させる
      publishMonitorStatus(SERVER_SERVICE_NAME, WARN, SERVER_DISCONNECTING_MSG);
    }
    else
    {
      //通信は失敗しているが設定値以下なので正常を通知
      publishMonitorStatus(SERVER_SERVICE_NAME, NOMAL, SERVER_CONNECTING_MSG);
    }
     
  }
  else
  {
    //通信成功中は正常を通知
    publishMonitorStatus(SERVER_SERVICE_NAME, NOMAL, SERVER_CONNECTING_MSG);
  }
}

// メインループ処理
//  ループ間隔は Safety waipoint のループより早い間隔が望ましい
//
void main_loop_callback(const ros::TimerEvent &)
{
  check_connection();

  carctl_msgs::RouteCheck r_flg_;
  int r;
  int arg;
  int ret;
  pthread_t thread1;

  // 現在のwaypoint取得
  getRouteCheckInfo(r_flg_);

  //checksrvで認識しているwaypointをpublish
  pub_checksrv_waypointid.publish(r_flg_);

  if (onWaitingPoint) {
    ROS_WARN("main_loop_callback: onWaitingPoint");
    return;
  }

  switch (r_flg_.RouteCheckFlg)
  {
  case 0: // 停止ポイント
    ROS_WARN("main_loop_callback: waiting point loop start waypoint[%d]", r_flg_.WaypointId);
    // 上位からの許可を停止状態にする
    SrvStopFlg();

    r = pthread_mutex_trylock(&mymutex);
    if ((r != 0) || (EBUSY == r)) {
      onWaitingPoint = false; // 走行ポイント確認用スレッドが動いている状態で停止位置に入るとロックが維持され、SrvStopFlg()により、メインループ内の停止位置確認用フローを再度呼び出せなくなるため、毎回フラグを解除する必要がある。
      break;
    }
    pthread_mutex_unlock(&mymutex);

    ROS_WARN("publish main");
    pub_checkpost.publish(r_flg_);
    break;
  case 1: // 走行ポイント
    // 停止判定なし
    // POST間隔を調整
    if (!check_elapsed_time(process_rate)) { break; }

      // 走行可
    SrvStartFlg();
    ROS_WARN("main_loop_callback: running point loop start waypoint[%d]", r_flg_.WaypointId);

    // 上位へPOST送信する
    r = pthread_mutex_trylock(&mymutex);
    if ((r != 0) || (EBUSY == r)) { break; }

    // trylock でロックを取得したので unlock する
    pthread_mutex_unlock(&mymutex);
    // 上位通信は別スレッドで送信する
    arg = r_flg_.WaypointId;
    g_arg = r_flg_.WaypointId;

    // fprintf(stdout, "DEBUG: main_loop_callback: arg=[%d]\n", arg);
    ret = pthread_create(&thread1, NULL, check_route_change_thread, (void *)&arg);
    // join による後始末を放棄してスレッド終了と同時にリソース解放はシステムに任せる
    pthread_detach(thread1);
    // 送信間隔の調整
    last_send_time = calc_time();

    break;
  default: // 未初期化状態
    break;
  }

  forcedPassWaitingWaypoint(process_wpoint_threshold);
}

//上位連携以外正常な状態で処理を開始するためのフラグ
void ctlvehicleCallBack(const std_msgs::Int16 &cvf)
{
  CVFlg_ = cvf.data;
}
//機台車両ボタンによる強制走行
void GpioStartCallBack(const std_msgs::Int16 &flg)
{
  if (flg.data == ON)
  {
    //発進ボタンが押されて走行する場合上位通信途絶状態を解除する。
    //通信途絶で停止する場合、停止箇所以外で停止する事がほとんどになるため、GSFlg_の更新タイミングとは分ける必要がある
    is_post_disconnect = false;
  }

  carctl_msgs::RouteCheck r_flg_;
  getRouteCheckInfo(r_flg_);

  // 停止位置でOFFに切り替わるのを防止する。理由はctlvehicle側でOFFにしてくることがあり、checksrv側としては意図しない状況でフラグ解除されることがあるため。
  // 状況によっては、停止位置用のループが解除されない事があった。
  if (r_flg_.RouteCheckFlg == 0/*停止位置*/ && flg.data != ON) { return; }
  GSFlg_ = flg.data;
}

void errorCallBack(const std_msgs::String &msg)
{
  error_id = msg.data;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose) {
  onWaitingPoint = false;
  forceClearTransitRecord();
}


std::thread init_check_post_callback(ros::NodeHandle &n)
{
  
  std::thread th([&]() {
      // キューを分けた場合
      ros::NodeHandle n_sep_queue;
      std::shared_ptr<ros::CallbackQueue> sep_queue = std::make_shared<ros::CallbackQueue>();

      n_sep_queue.setCallbackQueue(sep_queue.get());
      ros::Subscriber sub_check_post = n_sep_queue.subscribe("CheckPost", 1, check_post_callback);

      while (ros::ok()) {
          sep_queue->callAvailable(ros::WallDuration()); // 一定周期で呼び出しが必須
          ros::Duration(0.01).sleep();
      }
  });
  return th;
}


void batteryStatusCallback(const carctl_msgs::battery_status &msg) {
  battery_scale = std::to_string(msg.scale);
  battery_color = std::to_string(msg.color);
}


int main(int argc, char **argv)
{
  // fprintf(stdout, "checksrv start.\n");

  // ros 初期化
  ros::init(argc, argv, "checksrv");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  // ros parameter settings
  private_nh.getParam("post_url", post_url);
  ROS_DEBUG("post_url[%s]", post_url.c_str());

  private_nh.getParam("post_timeout", post_timeout);
  ROS_DEBUG("post_timeout[%d]", post_timeout);

  private_nh.getParam("post_force_timeout", post_force_timeout);
  ROS_DEBUG("post_force_timeout[%d]", post_force_timeout);

  private_nh.getParam("car_vehicleId", car_vehicleId);
  ROS_DEBUG("car_vehicleId[%s]", car_vehicleId.c_str());

  private_nh.getParam("car_Iface", car_Iface);
  ROS_DEBUG("car_Iface[%s]", car_Iface.c_str());

  private_nh.getParam("process_rate", process_rate);
  ROS_DEBUG("process_rate[%If]", process_rate);

  private_nh.getParam("process_wpoint_threshold", process_wpoint_threshold);
  ROS_DEBUG("process_wpoint_threshold[%d]", process_wpoint_threshold);

  private_nh.getParam("process_loop_timer", process_loop_timer);
  ROS_DEBUG("process_loop_timer[%If]", process_loop_timer);

  private_nh.getParam("disconnect_time_threshold", disconnect_time_threshold);
  ROS_DEBUG("disconnect_time_threshold[%If]", disconnect_time_threshold);

  private_nh.getParam("test_drive", test_drive);
  ROS_DEBUG("test_drive[%d]", test_drive);

  private_nh.getParam("post_disconnect_decision", post_disconnect_decision);
  ROS_DEBUG("post_disconnect_decision[%If]", post_disconnect_decision);

  //  }

  // ネットワークインターフェースに設定されているIPアドレスを取得
  memset(str_ip_addr, 0x00, sizeof(str_ip_addr));
  get_ip_address_string(car_Iface.c_str(), str_ip_addr);

  // post(curl)初期化
  init_post_vms_control(post_url.c_str(), post_timeout);
  // pub
  pub_SrvStartFlg = n.advertise<std_msgs::Int16>("SrvStartFlg", 10);
  pub_SrvStopFlg = n.advertise<std_msgs::Int16>("SrvStopFlg", 10);
  pub_ConnectStartFlg = n.advertise<std_msgs::Int16>("ConnectStartFlg", 10);
  pub_ConnectStopFlg = n.advertise<std_msgs::Int16>("ConnectStopFlg", 10);
  pub_checkpost = n.advertise<carctl_msgs::RouteCheck>("CheckPost", 1);
  pub_routechange = n.advertise<std_msgs::Int8>("RouteChangeFlg", 10);
  pub_waitingfor = n.advertise<carctl_msgs::waiting_msg>("WaitingFor_Result", 10);

  pub_statecmd = n.advertise<carctl_msgs::monitor_status>("/monitor_status", 10);

  pub_checksrv_waypointid = n.advertise<carctl_msgs::RouteCheck>("/checksrv_waypointid", 10);

  // sub
  ros::Subscriber sub_plcsensor = init_plc_sensor_packet_callback(n);
  //ros::Subscriber sub_waypoints = init_safety_waypoints_callback(n);
  std::thread th = init_safety_waypoints_callback(n);
  //ros::Subscriber sub_check_post = n.subscribe("CheckPost", 1, check_post_callback);
  std::thread check_post_th = init_check_post_callback(n);
  ros::Subscriber sub_chkvehicleflg = n.subscribe("ctlvehicle_flg", 10, ctlvehicleCallBack);
  //車両機台スイッチからの走行指示（強制走行）
  ros::Subscriber sub_GpioStart = n.subscribe("GpioStartFlg", 10, GpioStartCallBack);
  ros::Subscriber sub_error = n.subscribe("send_error_status", 10, errorCallBack);
  ros::Subscriber sub_initialpose = n.subscribe("/initialpose", 10, initialPoseCallback);
  ros::Subscriber sub_battery_status = n.subscribe("/battery_status", 10, batteryStatusCallback);

  // メインループ開始
  ros::Timer timer = n.createTimer(ros::Duration(process_loop_timer), main_loop_callback);

  ros::spin();

  th.join();
  check_post_th.join();

  // post(curl)クリーンアップ
  cleanup_post_vms_control();

  // fprintf(stdout, "DEBUG: checksrv end.\n");

  return 0;
}
