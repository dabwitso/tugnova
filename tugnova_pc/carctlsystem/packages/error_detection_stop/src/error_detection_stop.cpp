#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/TwistStamped.h>


//速度情報をパブリッシュするための変数
ros::Publisher pub_twist_out;

//異常検知フラグ
int16_t error_detection_flg = 0;

void twistCmdCallback(const geometry_msgs::TwistStamped &TwistStamped)
{
  // 目標速度(m/s)
  double twistVel = TwistStamped.twist.linear.x;

  //　異常検知フラグが0の場合速度情報を変更せずに車両に横流しする
  if (error_detection_flg == 0)
  {
    pub_twist_out.publish(TwistStamped);
  }
  else
  {
    //異常を検知している場合車両を停止
    geometry_msgs::TwistStamped newtwist = TwistStamped;
    newtwist.twist.linear.x = 0;
    pub_twist_out.publish(newtwist);
    //一度停止した後は異常検知フラグが更新されないため、

  }

  ros::spinOnce();
}

void errorDetectionFlgCallback(const std_msgs::Int16 &edf)
{
  //異常検知フラグが停止で更新された場合は起動バッチを再起動するまで走行させないため、フラグ更新処理を行わない
  if (error_detection_flg == 0)
  {
    error_detection_flg = edf.data;
  }
  
}

int main(int argc, char **argv)
{
  fprintf(stdout, "error_detection_stop start.\n");

  // ros 初期化
  ros::init(argc, argv, "error_detection_stop");
  ros::NodeHandle n;

  std::string input_twist_cmd_topic;
  std::string output_twist_cmd_topic;
  std::string error_detection_flg_topic;

  
  //速度情報をsubscribeするトピック
  n.param<std::string>("input_twist_cmd_topic", input_twist_cmd_topic, "/twist_cmd");
  //速度情報をpublishするトピック
  n.param<std::string>("output_twist_cmd_topic", output_twist_cmd_topic, "/out_twist_cmd");
  //エラーフラグのトピック
  n.param<std::string>("error_detection_flg_topic", error_detection_flg_topic, "/error_detection_flg");

  // pub
  pub_twist_out = n.advertise<geometry_msgs::TwistStamped>(output_twist_cmd_topic, 10);

  ros::Subscriber sub_twist_cmd = n.subscribe(input_twist_cmd_topic, 10, twistCmdCallback);
  ros::Subscriber sub_error_detection_flg = n.subscribe(error_detection_flg_topic, 10, errorDetectionFlgCallback);

  ros::spin();

  fprintf(stdout, "DEBUG: error_detection_stop end.\n");

  return 0;
}
