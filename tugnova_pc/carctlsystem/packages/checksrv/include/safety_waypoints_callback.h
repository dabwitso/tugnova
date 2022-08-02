#ifndef safety_waypoints_callback_h
#define safety_waypoints_callback_h

#include <vector>
#include "autoware_msgs/LaneArray.h"
#include "carctl_msgs/RouteCheck.h"
#include <transit_record_helper.h>

#include <thread>
#include <ros/callback_queue_interface.h>
#include <ros/callback_queue.h>
#include <util.h>

//
// safety waipoints のコールバック処理を共通化する
// グローバル変数を用意して随時参照してもらう
// 実装ロジック及びcallback処理自体はここに隠ぺいする
//

// 停止用
autoware_msgs::Lane obs_wp;

CheckSrv::TransitRecordHelper transitRecordHelper;
int previous_waypoint_id = -1;
int previous_route_check_flg = -1;

pthread_mutex_t waypoint_list_mutex = PTHREAD_MUTEX_INITIALIZER;

double connection_check_point_time = 0.0;

// コールバック関数
//  Autowareからの情報をSubして参照用の変数へ格納する
void safety_waypoints_callback(const autoware_msgs::Lane &lane)
{
  ROS_WARN("safety_waypoints_callback");
  pthread_mutex_lock(&waypoint_list_mutex);
  autoware_msgs::WaypointCustom now_wpc_;
  autoware_msgs::Waypoint wp_ = lane.waypoints[0];

  now_wpc_ = wp_.wpc;
  ROS_WARN("safety_waypoints_callback waypoint=[%d]", now_wpc_.waypoint_id);

  transitRecordHelper.push(now_wpc_.waypoint_id, now_wpc_.waiting_state);

  //上位通信確認地点に到達した場合、通信確認地点到達時刻を設定
  if (now_wpc_.connection_check == 1)
  {
    if (connection_check_point_time == 0.0)
    {
      ROS_WARN("connection_check_point_time update");
      connection_check_point_time = calc_time();
    }
  }

  // fprintf(stdout, "DEBUG: safety_waypoints_callback: WaypointId=[%d] RouteCheckFlg=[%d]\n", now_wpc_.waypoint_id, (now_wpc_.waiting_state == 0 ? 1 : 0));
  fflush(stdout);
  pthread_mutex_unlock(&waypoint_list_mutex);
}

//上位通信確認地点到達時刻を取得
double getConnectionCheckPointTime()
{
  return connection_check_point_time;
}

//上位通信確認地点到達時刻を設定
void setConnectionCheckPointTime(double time)
{
  connection_check_point_time = time;
}

// 現在時点のWaypoint情報を取得する
//  i/o : n メイン側で定義したルートチェックの参照
void getRouteCheckInfo(carctl_msgs::RouteCheck &target_)
{
  pthread_mutex_lock(&waypoint_list_mutex);
  CheckSrv::TransitRecord::Ptr target = transitRecordHelper.getTargetWaypoint();
  if (target == NULL) {
    target_.WaypointId = previous_waypoint_id;
    target_.RouteCheckFlg = (previous_route_check_flg == -1) ? -1 : 1; // リストがクリアされた直後という事は、発進許可が出たためであるため、停止地点を発進地点と見なす処理を施すことで、即停止する挙動を防止する
    pthread_mutex_unlock(&waypoint_list_mutex);
    return;
  }

  target_.WaypointId = previous_waypoint_id = target->getWaypointId();
  if (target->getWaitingState() == 0) {
    target_.RouteCheckFlg = previous_route_check_flg = 1;
  } else {
    target_.RouteCheckFlg = previous_route_check_flg = 0;
  }
  pthread_mutex_unlock(&waypoint_list_mutex);
}

/**
 * 通過した停止位置を記録.
 * ※pass系関数とclear系関数の違いについて。clear系は「上位との通信が完了した後」に行う行為で、pass系は「上位は関係なく通過した事だけ記録したい」時に行う。
 */
void passWaitingWaypoint(const int waypoint_id) {
  pthread_mutex_lock(&waypoint_list_mutex);
  transitRecordHelper.passWaitingWaypoint(waypoint_id);
  pthread_mutex_unlock(&waypoint_list_mutex);
}

/**
 * 現在位置から見て、閾値以上、通過した場合、強制的に通過したと見なす.
 */
void forcedPassWaitingWaypoint(const int n_th_later) {
  // 後始末
  //  STOPポイント対象として現在のwaypointから閾値の±ｎを超えているものは削除する
  //  これは停止ポイントから手動で車を移動させたときの後始末を想定している
  //  もしくは車をバックさせたときに、再度停止ポイントで停止させることを想定している。
  //  停止ポイントと現在ポイントの差は設定により変更可能。（way point ID依存）
  pthread_mutex_lock(&waypoint_list_mutex);
  transitRecordHelper.forcedPassWaitingWaypoint(n_th_later);
  pthread_mutex_unlock(&waypoint_list_mutex);
}

/**
 * 通過したWaypointIDの一覧を取得.
 */
std::vector<int> getTransitRecordList() {
  pthread_mutex_lock(&waypoint_list_mutex);
  std::vector<int> list = transitRecordHelper.getList();
  pthread_mutex_unlock(&waypoint_list_mutex);
  return list;
}

/**
 * 通過したWaypointIDの一覧をクリア.
 * 送信前のWaypointIDは残る.
 */
void clearTransitRecord() {
  pthread_mutex_lock(&waypoint_list_mutex);
  transitRecordHelper.clear(-1);
  pthread_mutex_unlock(&waypoint_list_mutex);
}

/**
 * 通過したWaypointIDの一覧をクリアするが、指定したWaypointは残す.
 * 送信前のWaypointIDは残る.
 */
void clearTransitRecordOtherExceptionWaypointID(const int waypoint_id) {
  pthread_mutex_lock(&waypoint_list_mutex);
  transitRecordHelper.clear(waypoint_id);
  pthread_mutex_unlock(&waypoint_list_mutex);
}

/**
 * WaypointIDの一覧を完全クリア.
 */
void forceClearTransitRecord() {
  previous_waypoint_id = -1;
  previous_route_check_flg = -1;
  transitRecordHelper.reserveClear();
}

// safety waipoints のコールバック 初期化処理
//  i : n メイン側で定義したノードハンドルの参照
//  return : rosに登録したSubscriberオブジェクト
std::thread init_safety_waypoints_callback(ros::NodeHandle &n)
{
  pthread_mutex_init(&waypoint_list_mutex, NULL);
  // fprintf(stdout, "DEBUG: safety_waypoints_callback: 初期化実行\n");
  //return n.subscribe("safety_waypoints", 10, safety_waypoints_callback);
  std::thread th([&]() {
      // キューを分けた場合
      ros::NodeHandle n_sep_queue;
      std::shared_ptr<ros::CallbackQueue> sep_queue = std::make_shared<ros::CallbackQueue>();

      n_sep_queue.setCallbackQueue(sep_queue.get());
      ros::Subscriber sub_waypoints = n_sep_queue.subscribe("safety_waypoints", 10, safety_waypoints_callback);

      while (ros::ok()) {
          sep_queue->callAvailable(ros::WallDuration()); // 一定周期で呼び出しが必須
          ros::Duration(0.01).sleep();
      }
  });
  return th;
}

void WaypointsCallback(const autoware_msgs::Lane &lane)
{
  obs_wp = lane;
}

#endif // safety_waypoints_callback_h
