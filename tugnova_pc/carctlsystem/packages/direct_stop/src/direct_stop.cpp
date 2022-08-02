#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/TwistStamped.h>

//速度情報をパブリッシュするための変数
ros::Publisher pub_twist_out;
ros::Subscriber sub_twist_cmd;
ros::Subscriber sub_error_flg;
ros::Subscriber sub_stop_flg;

std::string input_twist_cmd_topic;
std::string output_twist_cmd_topic;
std::string stop_flg_topic;
std::string error_flg_topic;

const int16_t DIRECT = 0;
const int16_t RUN = 1;
const int16_t ERROR = 2;

//異常検知フラグ
int16_t stop_flg = DIRECT;

void twistCmdCallback(const geometry_msgs::TwistStamped &TwistStamped)
{
  // 目標速度(m/s)
  double twistVel = TwistStamped.twist.linear.x;

  geometry_msgs::TwistStamped newtwist;

  //　停止検知フラグが0の場合速度情報を変更せずに車両に横流しする
  switch (stop_flg)
  {
  case RUN:
    pub_twist_out.publish(TwistStamped);
    fprintf(stdout, "走行\n");
    break;
  //エラーでも直接停止でも同じように停止させる
  case ERROR:
  case DIRECT:
    fprintf(stdout, "停止を検知している場合車両を停止\n");
    fprintf(stdout, "stop_flg:%d\n", stop_flg);
    //停止を検知している場合車両を停止
    newtwist = TwistStamped;
    newtwist.twist.linear.x = 0;
    pub_twist_out.publish(newtwist);
    break;

  default:
    break;
  }

  ros::spinOnce();
}

void updateStopFlg(const std_msgs::Int16 &flg)
{
  //エラーでなかった場合はフラグを更新する
  if (stop_flg != ERROR)
  {
    stop_flg = flg.data;
  }
}

void errorDetectionFlgCallback(const std_msgs::Int16 &edf)
{
  updateStopFlg(edf);
}

void stopFlgCallback(const std_msgs::Int16 &sf)
{
  updateStopFlg(sf);
}

int main(int argc, char **argv)
{
  fprintf(stdout, "direct_stop start.\n");

  // ros 初期化
  ros::init(argc, argv, "direct_stop");
  ros::NodeHandle n;

  std::string input_twist_cmd_topic;
  std::string output_twist_cmd_topic;
  std::string stop_flg_topic;
  std::string error_flg_topic;

  //速度情報をsubscribeするトピック
  n.param<std::string>("/direct_stop/input_twist_cmd_topic", input_twist_cmd_topic, "/twist_cmd");
  //速度情報をpublishするトピック
  n.param<std::string>("/direct_stop/output_twist_cmd_topic", output_twist_cmd_topic, "/out_twist_cmd");
  //エラーフラグのトピック
  n.param<std::string>("/direct_stop/error_flg_topic", error_flg_topic, "/error_flg");
  //停止フラグのトピック
  n.param<std::string>("/direct_stop/stop_flg_topic", stop_flg_topic, "/stop_flg");

  // pub
  pub_twist_out = n.advertise<geometry_msgs::TwistStamped>(output_twist_cmd_topic, 10);

  sub_twist_cmd = n.subscribe(input_twist_cmd_topic, 10, twistCmdCallback);
  sub_error_flg = n.subscribe(error_flg_topic, 10, errorDetectionFlgCallback);
  sub_stop_flg = n.subscribe(stop_flg_topic, 10, stopFlgCallback);

  ros::spin();

  fprintf(stdout, "DEBUG: direct_stop end.\n");

  return 0;
}
