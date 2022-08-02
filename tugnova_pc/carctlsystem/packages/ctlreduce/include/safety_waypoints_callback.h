#ifndef safety_waypoints_callback_h
#define safety_waypoints_callback_h

#include "autoware_msgs/LaneArray.h"
#include "carctl_msgs/RouteCheck.h"

//
// safety waipoints のコールバック処理を共通化する
// グローバル変数を用意して随時参照してもらう
// 実装ロジック及びcallback処理自体はここに隠ぺいする
//

// Waypoint＋停止フラグ　参照用
carctl_msgs::RouteCheck route_chk_flg;
// 停止用
autoware_msgs::Lane obs_wp;

// コールバック関数
//  Autowareからの情報をSubして参照用の変数へ格納する
void safety_waypoints_callback(const autoware_msgs::Lane &lane)
{
  autoware_msgs::WaypointCustom now_wpc_;
  autoware_msgs::Waypoint wp_ = lane.waypoints[0];

  now_wpc_ = wp_.wpc;

  if (now_wpc_.waiting_state == 0)
  {
    route_chk_flg.RouteCheckFlg = 1;
  }
  else
  {
    route_chk_flg.RouteCheckFlg = 0;
  }
  route_chk_flg.WaypointId = now_wpc_.waypoint_id;

  fprintf(stdout, "DEBUG: safety_waypoints_callback: WaypointId=[%d] RouteCheckFlg=[%d]\n", route_chk_flg.WaypointId, route_chk_flg.RouteCheckFlg);
  fflush(stdout);
}

// 現在時点のWaypoint情報を取得する
//  i/o : n メイン側で定義したルートチェックの参照
void getRouteCheckInfo(carctl_msgs::RouteCheck &target_)
{
  target_.WaypointId = route_chk_flg.WaypointId;
  target_.RouteCheckFlg = route_chk_flg.RouteCheckFlg;
}

// safety waipoints のコールバック 初期化処理
//  i : n メイン側で定義したノードハンドルの参照
//  return : rosに登録したSubscriberオブジェクト
ros::Subscriber init_safety_waypoints_callback(ros::NodeHandle &n)
{
  fprintf(stdout, "DEBUG: safety_waypoints_callback: 初期化実行\n");
  route_chk_flg.WaypointId = -1;
  route_chk_flg.RouteCheckFlg = -1;
  return n.subscribe("safety_waypoints", 10, safety_waypoints_callback);
}

void WaypointsCallback(const autoware_msgs::Lane &lane)
{
  obs_wp = lane;
}

#endif // safety_waypoints_callback_h
