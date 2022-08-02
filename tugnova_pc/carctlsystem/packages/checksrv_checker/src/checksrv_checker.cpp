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
#include "carctl_msgs/RouteCheck.h"

carctl_msgs::RouteCheck current_checksrv_waypoint;

//タイムアウト待ち中にchecksrv_waypointが来ないため最後に受信した時間から
double waypoint_update_time = 0.0;
double error_run_start_time = 0.0;
double run_threshold = 3.0;

double waypoint_update_threshold = 3.0;

ros::Publisher pub_statecmd;

//異常状態
const int16_t NOMAL = 0;
const int16_t ERROR = 1;
const int16_t WARN = 2;

bool is_error_occurred = false;
bool is_waypoint_update = true;

const std::string SERVICE_NAME = "checksrv_checker";
const std::string ERROR_ID = "checksrv_1_0";
const std::string NOMAL_ID = "checksrv_0_0";

// 現在時刻をミリ秒で取得する
double calc_time()
{
  struct ::timespec getTime;
  clock_gettime(CLOCK_MONOTONIC, &getTime);
  return (getTime.tv_sec + getTime.tv_nsec * 1e-9) * 1000;
}

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

void checksrv_waypointid_callback(const carctl_msgs::RouteCheck &new_checksrv_waypoint)
{
  ROS_WARN("checksrv_waypointid_callback: start current_checksrv_waypoint.WaypointId[%d], new_checksrv_waypoint[%d]", current_checksrv_waypoint.WaypointId, new_checksrv_waypoint.WaypointId);
  //最後にwaypointが更新された時刻を退避
  waypoint_update_time = calc_time();

  if (current_checksrv_waypoint.WaypointId != new_checksrv_waypoint.WaypointId)
  {
    //waypointが変更された場合
    current_checksrv_waypoint = new_checksrv_waypoint;
    is_waypoint_update = true;
    is_error_occurred = false;
    //異常走行開始時間を初期化
    error_run_start_time = 0.0;
    //正常時のステータス出力
    publishMonitorStatus(SERVICE_NAME, NOMAL, NOMAL_ID);
  }
  else
  {
    //waypointが更新されない場合
    is_waypoint_update = false;
  }
}

void plc_sensor_packet_Callback(const udp_msgs::UdpSensorPacket &coms_data)
{
  ROS_WARN("plc_sensor_packet_Callback: start is_waypoint_update[%d], is_error_occurred[%d], coms_data.PLC_vel[%lf], coms_data.ECUMode[%d]", is_waypoint_update, is_error_occurred, coms_data.PLC_vel, coms_data.ECUMode);

  if (coms_data.ECUMode == 0)
  {
    //手動運転モードの場合はエラー判定を行わない
    //異常走行開始時間を初期化
    error_run_start_time = 0.0;
    return;
  }

  if (coms_data.PLC_vel > 0.0)
  {
    //車両が走行している場合

    if (is_waypoint_update)
    {
      //waypointが更新されている場合
      //異常走行開始時間を初期化
      error_run_start_time = 0.0;

      double waypoint_update_diff_time = calc_time() - waypoint_update_time;
      ROS_WARN("plc_sensor_packet_Callback: waypoint_update_diff_time[%lf]", waypoint_update_diff_time);
      if ((waypoint_update_threshold * 1000.0) <= waypoint_update_diff_time)
      {
        //waypointが一定時間更新されない場合にwaypointが更新されていないと判断する
        is_waypoint_update = false;
      }
    }
    else
    {
      //waypointが更新されていない場合
      if (error_run_start_time == 0.0)
      {
        //異常走行開始時間が初期値の場合、走行開始時間を記録
        error_run_start_time = calc_time();
      }
      else
      {
        //異常走行開始時間に値が存在する場合、走行時間をカウント
        double error_run_start_diff_time = calc_time() - error_run_start_time;
        ROS_WARN("plc_sensor_packet_Callback: error_run_start_diff_time[%lf]", error_run_start_diff_time);
        if ((run_threshold * 1000.0) <= error_run_start_diff_time)
        {
          //閾値時間以上走行している場合
          //エラー出力
          is_error_occurred = true;
          publishMonitorStatus(SERVICE_NAME, ERROR, ERROR_ID);
          ROS_WARN("plc_sensor_packet_Callback: checksrv_1_0 error run_start_time");
        }
        else
        {
          //閾値時間以上走行していない場合
          //正常時のステータス出力
          publishMonitorStatus(SERVICE_NAME, NOMAL, NOMAL_ID);
        }
      }
    }
  }
  else
  {
    //車両が停止している場合
    //異常走行開始時間を初期化
    error_run_start_time = 0.0;

    if (is_error_occurred)
    {
      //エラーが発生している場合
      //エラー停止後にもエラーを出力し続ける
      publishMonitorStatus(SERVICE_NAME, ERROR, ERROR_ID);
      ROS_WARN("plc_sensor_packet_Callback: checksrv_1_0 error is_error_occurred");
    }
    else
    {
      //エラーが発生していない場合
      //正常時のステータス出力
      publishMonitorStatus(SERVICE_NAME, NOMAL, NOMAL_ID);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "checksrv_checker");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle n;

  private_nh.getParam("run_threshold", run_threshold);
  private_nh.getParam("waypoint_update_threshold", waypoint_update_threshold);

  //初期化
  current_checksrv_waypoint.WaypointId = 0;
  current_checksrv_waypoint.RouteCheckFlg = -1;

  pub_statecmd = n.advertise<carctl_msgs::monitor_status>("/monitor_status", 10);

  // sub
  ros::Subscriber sub_checksrv_waypointid = n.subscribe("checksrv_waypointid", 10, checksrv_waypointid_callback);
  ros::Subscriber sub_plcsensor = n.subscribe("plc_sensor_packet", 10, plc_sensor_packet_Callback);

  ros::spin();

  return 0;
}
