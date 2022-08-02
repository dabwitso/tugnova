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

autoware_msgs::WaypointCustom now_wpc;
ros::Publisher pub_stopflg;
ros::Publisher statecmd;

//ルート変更なし
const int CONTINUE = 0;
//分割された次のルートを読み込む
const int CHANGE = 1;
//分割された次のルートを読み込む。サーバーではCHANGE_ONLYの王場合、次のルートを作成しない。車両の処理はCHANGEと変わらない。
const int CHANGE_ONLY = 2;
//上位サーバーに問い合わせを行い、ダウンロードしたルートで走行
const int SERVER = 3;

char *ROUTE_ID_FILE = "/home/nvidia/Autoware/current_lane_id";
char *ROUTE_PATH = "/home/nvidia/Autoware/ros/renkei";

//ルート切り替え時のコマンド
std::string change_route_cmd_str = "/home/nvidia/Autoware/ros/nodeshell/ChangeRoute.sh ";

bool run_flg = false;

//前回経路切り替え処理を行ったwaypoint
int exe_waypoint = 0;

const int16_t STOP = 0;
const int16_t RUN = 1;
// const int16_t ERROR = 2;

//異常状態
const int16_t NOMAL = 0;
const int16_t ERROR = 1;

time_t timer;

char sprit_char = '_';

//異常検知機能に異常を通知
void publishMonitorStatus(int16_t status, std::string error_msg)
{
  carctl_msgs::monitor_status msg;
  //ノード名は固定
  msg.service_name = "check_change_route";
  //状態を格納
  msg.status = status;
  //エラーメッセージを格納
  msg.error_msg = error_msg;
  //ログ出力
  if (status == NOMAL)
  {
     ROS_INFO("Infomation: [%s] \n", error_msg.c_str());
  }
  else
  {
     ROS_ERROR("Infomation: [%s] \n", error_msg.c_str());
  }
  //パブリッシュ
  statecmd.publish(msg);

  ros::spinOnce();
}

//車両を走行または停止させる
void PublishStopFlg(int16_t order_)
{
  std_msgs::Int16 msg;
  msg.data = order_;
  pub_stopflg.publish(msg);
  ros::spinOnce();
  return;
}

//文字列を分割する
std::vector<std::string> split(std::string &str, char sep)
{
  std::vector<std::string> v;
  std::stringstream ss(str);
  std::string buffer;
  while (std::getline(ss, buffer, sep))
  {
    v.push_back(buffer);
  }

  return v;
}

//引数の文字列を連結し、次に読み込むルートのidを返す
std::string getNextLaneID(std::string course_id, int route_count)
{
  std::stringstream ss;
  ss << course_id << sprit_char << route_count;
  return ss.str();
}

//次に読み込むルートのid（next_lane_id）のルートファイルが存在するか確認する。呼び出し元でnext_lane_idを分割しているためメソッド内では分割せず引数でもらう
bool isExsistRouteFile(std::string next_lane_id)
{
  std::vector<std::string> lane_id_str = split(next_lane_id, sprit_char);

  std::stringstream nf;
  nf << ROUTE_PATH << "/" << lane_id_str[0] << "/"
     << "ROUTE" << "/"
     << lane_id_str[1] << "/" << next_lane_id << ".csv";
  std::string nfp = nf.str();
  const char *next_file_path = nfp.c_str();
  std::ifstream route_file(next_file_path);
  return route_file.is_open();
}

//枝番をカウントアップしてルート読込みバッチを起動する
void changeLocalRoute()
{
  if (run_flg == false)
  {
    run_flg = true;
    time(&timer);
    fprintf(stdout, "ルート変更開始:%s\n", ctime(&timer));

    //ファイルから現在読み込んでいるルートIDを取得
    std::ifstream route_ifs(ROUTE_ID_FILE);
    std::string current_lane_id;

    //current_lane_idファイルチェック
    if (!route_ifs.is_open())
    {
      //車両をエラー停止させる
      // PublishStopFlg(ERROR);
      fprintf(stdout, "current_lane_idファイルが不正です\n");
      publishMonitorStatus(ERROR, "Cannot open file. [current_lane_id]");
      return;
    }

    getline(route_ifs, current_lane_id);
    fprintf(stdout, "current_lane_id:%s\n", current_lane_id.c_str());
    route_ifs.close();

    std::vector<std::string> lane_id_str = split(current_lane_id, sprit_char);

    //lane_id_strの長さが2でない場合エラー
    if (lane_id_str.size() != 2)
    {
      //車両をエラー停止させる
      // PublishStopFlg(ERROR);
      fprintf(stdout, "current_lane_idファイルが不正です。　フォーマットエラー\n");
      publishMonitorStatus(ERROR, "The file format is invalid, lane_id_str length error. [current_lane_id]");
      return;
    }

    //枝番を抽出する
    int route_count = atoi(lane_id_str[1].c_str());

    //atoiの引数に文字が入力された場合0が出力されるためその場合エラーにする。枝番0はありえないため問題なし
    if (route_count == 0)
    {
      //車両をエラー停止させる
      // PublishStopFlg(ERROR);
      fprintf(stdout, "current_lane_idファイルが不正です。　ルート枝番が文字もしくは0\n");
      publishMonitorStatus(ERROR, "The file format is invalid. [current_lane_id]");
      return;
    }

    //ルートIDの枝番をカウントアップ
    route_count++;

    //読み込む次のルートファイル名設定
    std::string next_lane_id = getNextLaneID(lane_id_str[0], route_count);

    fprintf(stdout, "next_lane_id:%s\n", next_lane_id.c_str());

    //ルートファイルの存在チェック
    if (!isExsistRouteFile(next_lane_id))
    {
      //次ルートのルートファイルが存在しない場合、枝番1のルートファイルを読み込む
      fprintf(stdout, "次ルートファイルが存在しないため、枝番1のルートファイルを読み込みます。\n");
      //ルートIDの枝番を1にする
      route_count = 1;

      //読み込む次のルートファイル名設定
      next_lane_id = getNextLaneID(lane_id_str[0], route_count);

      fprintf(stdout, "next_lane_id:%s\n", next_lane_id.c_str());

      if (!isExsistRouteFile(next_lane_id))
      {
        //車両をエラー停止させる
        fprintf(stdout, "読み込むルートファイルが不正です。\n");
        publishMonitorStatus(ERROR, "読み込むルートファイルが不正です。");
        return;
      }
    }

    ros::NodeHandle n;
    n.setParam("current_lane_id", next_lane_id);

    //カウントアップしたルートIDでルート切り替えバッチを起動

    //車両を直接停止させる
    PublishStopFlg(STOP);
    //ルート切り替えコマンド作成
    std::string cmd_str = change_route_cmd_str + next_lane_id;
    const char *cmd = cmd_str.c_str();
    //ルート切り替えコマンド実行
    system(cmd);
    //車両を走行させる
    PublishStopFlg(RUN);

    run_flg = false;
    publishMonitorStatus(NOMAL, "route change end.");

    time(&timer);
    fprintf(stdout, "ルート変更終了:%s\n", ctime(&timer));
  }
}

void WaypointsCallback(const autoware_msgs::Lane &lane)
{
  now_wpc = lane.waypoints[0].wpc;
  int now_waypoint = now_wpc.waypoint_id;
  fprintf(stdout, "now_waypoint:%d\n", now_waypoint);
  fprintf(stdout, "exe_waypoint:%d\n", exe_waypoint);
  fprintf(stdout, "route_change:%d\n", now_wpc.route_change);
  fprintf(stdout, "run_flg:%d\n", run_flg);
  fprintf(stdout, "::::::::::\n");
  if (exe_waypoint == now_waypoint)
  {
    //ルート変更なし、かつ切り替え処理でない場合、走行する。
    if (now_wpc.route_change == CONTINUE && run_flg == false)
    {
      PublishStopFlg(RUN);
      publishMonitorStatus(NOMAL, "route no change.");
    }
    //それ以外で前回のwaypointIDと同じ場合処理を行わない
  }
  else
  {
    //前回のwaypointIDと違う場合ルート切り替え判定
    fprintf(stdout, "次のwaypointID:%d → %d\n", exe_waypoint, now_waypoint);
    switch (now_wpc.route_change)
    {
    case CONTINUE: //ルート変更なし
      //変更無い場合は走行を出力
      //切り替え箇所を超えた場合に走行にならないよう切り替え処理中は走行処理を行わない
      if (run_flg == false)
      {
        PublishStopFlg(RUN);
      }

      break;
    case CHANGE: //ルート変更
    case CHANGE_ONLY: //切り替えのみでも動作は同じ
      //ルート切り替えを実施するwaypointIDを退避
      exe_waypoint = now_waypoint;
      changeLocalRoute();
      break;
    case SERVER: //サーバーに問い合わせを行いルート変更
      break;
    default:
      break;
    }
    
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "check_change_route");
  ros::NodeHandle n;
  pub_stopflg = n.advertise<std_msgs::Int16>("RouteChangeCheckFlg", 10);
  statecmd = n.advertise<carctl_msgs::monitor_status>("/monitor_status", 10);
  //初回は走行でフラグを更新する
  PublishStopFlg(RUN);
  ros::Subscriber sub_waypoints = n.subscribe("safety_waypoints", 10, WaypointsCallback);
  ros::spin();
  return 0;
}
