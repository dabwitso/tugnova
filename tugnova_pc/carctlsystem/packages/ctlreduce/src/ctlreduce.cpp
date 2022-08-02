#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <carctl_msgs/battery_status.h>
#include <battery_checker.h>

#include <cstdlib>

#include "waypoint_follower/libwaypoint_follower.h"
#include <safety_waypoints_callback.h>

// パラメータ初期値
double process_SlowDown = 0.3;
double process_ReduceConst = 254;
double process_friction = 0.008;
int process_obssize = 30;
int16_t TlrReduceFlg = 0;
int battery_status = BATTERY_OK;

ros::Publisher pub_twistReduce;

// 直近の停止箇所もしくは信号のWaypointを取得
// PlcConverter::waypointCallbackで取得
// obs_wp : safty_waypoint
//  o : 停止ポイント
int GetStopPoint()
{
  int obssize;
  int Wait_state;
  int16_t tlrflg = TlrReduceFlg;

  // Waypointの個数を取得。obssizeで指定した距離まで停止箇所を確認する
  int max_obspoint = obs_wp.waypoints.size();
  if (max_obspoint < process_obssize)
  {
    obssize = max_obspoint;
  }
  else
  {
    obssize = process_obssize;
  }

  //  直近の停止箇所もしくは信号のWaypointを取得 -> stop_point
  //  PlcConverter::waypointCallbackで取得
  //  obs_wp : safty_waypoint

  for (int i = 0; i < obssize; i++)
  {
    // 信号機赤もしくは一時停止箇所による減速要否（走行可能：1,停止:0) およびtlr.range：信号機 : 4（停止位置）
    if (tlrflg == 0 && obs_wp.waypoints[i].wpc.tlr.range == 4)
    {
      fprintf(stdout, "DEBUG: GetStopPoint: 信号機停止位置を取得 stop_point[%d], TlrFlg[%d]\n", i, tlrflg);
      return i;
    }
    // waiting_state：停止箇所 : 1
    else if (obs_wp.waypoints[i].wpc.waiting_state == 1)
    {
      fprintf(stdout, "DEBUG: GetStopPoint: 上位停止箇所位置を取得 stop_point[%d]\n", i);
      return i;
    }
    // route_change：ルートチェンジ箇所：1,2
    else if (obs_wp.waypoints[i].wpc.route_change == 1 || obs_wp.waypoints[i].wpc.route_change == 2)
    {
      fprintf(stdout, "DEBUG: GetStopPoint: 経路切替箇所位置を取得 stop_point[%d]\n", i);
      return i;
    }
    // pause_point：一旦停止箇所：1
    else if (obs_wp.waypoints[i].wpc.pause_point == 1)
    {
      fprintf(stdout, "DEBUG: GetStopPoint: 一旦停止箇所位置を取得 stop_point[%d]\n", i);
      return i;
    }
    // バッテリ系では、「検知＆停止」地点への減速処理は行わない。仕様として「検知＆停止」地点は「停止位置」に必ず設置する、と決めたためである
    else if (battery_status == BATTERY_EXCHANGE && obs_wp.waypoints[i].wpc.battery_exchange == STOP_ONLY) {
      fprintf(stdout, "DEBUG: GetStopPoint: バッテリ交換停止位置を取得 stop_point[%d]\n", i);
      return i;
    }
    else if (battery_status == BATTERY_LOW && obs_wp.waypoints[i].wpc.battery_low == STOP_ONLY) {
      fprintf(stdout, "DEBUG: GetStopPoint: バッテリ警告停止位置を取得 stop_point[%d]\n", i);
      return i;
    }
  }
  fprintf(stdout, "DEBUG: GetStopPoint: 直近ルートに停止箇所なし\n");
  return -1; // 停止位置はなかった
}

// 停止箇所までの距離(m)から車速を割り出す
//  i : 停止ポイント
//  o : 車速(km/h)
double GetTwistReduce(int stopp)
{
  double stop_distance = 0.0;

  // 停止箇所までの距離(m)を計測 -> stop_distance
  //  PlcConverter::waypointCallbackで取得
  for (int i = stopp; i > 0; i--)
  {
    tf::Vector3 v1(obs_wp.waypoints[i].pose.pose.position.x,
                   obs_wp.waypoints[i].pose.pose.position.y, 0);
    tf::Vector3 v2(obs_wp.waypoints[i - 1].pose.pose.position.x,
                   obs_wp.waypoints[i - 1].pose.pose.position.y, 0);

    stop_distance += tf::tfDistance(v1, v2);
  }

  // 制動前の車速(km/h) = 平方根( 定数 × 摩擦係数 × 制動距離(m) ) / 3.6(kmph -> mps)
  return std::sqrt(process_ReduceConst * process_friction * stop_distance) / 3.6;
}

void TwistCmdCallback(const geometry_msgs::TwistStamped &TwistStamped)
{
  // 停止ポイント
  int stop_point = -1;
  // 目標速度(m/s)
  double twistVel = TwistStamped.twist.linear.x;

  // 信号機赤もしくは一時停止箇所による減速要否（走行可能：1,停止:0)
  // int16_t tlrflg = TlrReduceFlg;
  // if (tlrflg == 0)
  //{
  // 停止ポイントの取得
  //stop_point = GetStopPoint();
  //}

  // 停止ポイントの取得
  stop_point = GetStopPoint();

  // 徐行速度未満ならば現在の速度のまま
  if (stop_point == -1)
  {
    fprintf(stdout, "DEBUG: TwistCmdCallback: 直近停止箇所ないため減速なし twistVel[%2.1lf]\n", twistVel * 3.6);
    pub_twistReduce.publish(TwistStamped);
  }
  else if (stop_point == 0)
  {
    fprintf(stdout, "DEBUG: TwistCmdCallback: 停止箇所にて停止中\n");
    pub_twistReduce.publish(TwistStamped);
  }
  else if (twistVel < process_SlowDown)
  {
    // fprintf(stdout, "DEBUG: ctlreduce: 徐行 現在速度遅い twistVel[%2.1lf] stop_point[%d] tlrflg[%d]\n", twistVel * 3.6, stop_point,tlrflg);
    fprintf(stdout, "DEBUG: TwistCmdCallback: 最低速度以下のため減速なし twistVel[%2.1lf]\n", twistVel * 3.6);
    pub_twistReduce.publish(TwistStamped);
  }
  else
  {
    // 速度算出し、twistVel > preVel のとき減速する
    double preVel = GetTwistReduce(stop_point);
    if (twistVel > preVel)
    {
      fprintf(stdout, "DEBUG: TwistCmdCallback: 速度超過のため減速あり twistVel >  preVel [%2.1lf]->[%2.1lf]\n", twistVel * 3.6, preVel * 3.6);
      geometry_msgs::TwistStamped newtwist_ = TwistStamped;
      // 減速比率を計算
      double coefficient = preVel / twistVel;
      newtwist_.twist.linear.x = preVel;
      // 減速比率を角速度に適用
      newtwist_.twist.angular.z *= coefficient;
      pub_twistReduce.publish(newtwist_);
    }
    else
    {
      fprintf(stdout, "DEBUG: TwistCmdCallback: 減速なし twistVel <= preVel [%2.1lf]\n", twistVel * 3.6);
      pub_twistReduce.publish(TwistStamped);
    }
  }

  ros::spinOnce();
  fflush(stdout);
}

void TlrReduceFlgCallback(const std_msgs::Int16 &flg)
{
  TlrReduceFlg = flg.data;
}

/** バッテリ情報取得. */
void batteryStatusCallback(const carctl_msgs::battery_status &msg) {
  battery_status = msg.status;
}

int main(int argc, char **argv)
{
  fprintf(stdout, "ctlreduce start.\n");

  // ros 初期化
  ros::init(argc, argv, "ctlreduce");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  //if (private_nh.hasParam("/process/SlowDown") && private_nh.hasParam("/process/ReduceConst") && private_nh.hasParam("/process/friction") && private_nh.hasParam("/process/obssize"))
  //{
  private_nh.getParam("process_SlowDown", process_SlowDown);
  ROS_DEBUG("process_SlowDown[%lf]", process_SlowDown);

  private_nh.getParam("process_ReduceConst", process_ReduceConst);
  ROS_DEBUG("process_ReduceConst[%lf]", process_ReduceConst);

  private_nh.getParam("process_friction", process_friction);
  ROS_DEBUG("process_friction[%lf]", process_friction);

  private_nh.getParam("process_obssize", process_obssize);
  ROS_DEBUG("process_obssize[%d]", process_obssize);
  //}
  // pub
  pub_twistReduce = n.advertise<geometry_msgs::TwistStamped>("twistreduce_cmd", 10);
  // sub
  ros::Subscriber sub_waypoints = n.subscribe("safety_waypoints", 10, WaypointsCallback);
  ros::Subscriber sub_Phone = n.subscribe("twist_cmd", 10, TwistCmdCallback);
  ros::Subscriber sub_checkTlr = n.subscribe("TlrReduceFlg", 10, TlrReduceFlgCallback);
  ros::Subscriber sub_battery = n.subscribe("battery_status", 10, batteryStatusCallback);

  ros::spin();

  fprintf(stdout, "DEBUG: ctlreduce end.\n");

  return 0;
}
