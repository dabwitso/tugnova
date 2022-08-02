#include <ros/ros.h>
#include <sstream>
#include "autoware_msgs/LaneArray.h"
#include <std_msgs/Int16.h>
#include <std_msgs/String.h> 

#include <udp_msgs/UdpSensorPacket.h>

ros::Publisher pub_SrvStartFlg;
ros::Publisher pub_SrvStopFlg;
ros::Publisher pub_TlrStartFlg;
ros::Publisher pub_TlrStopFlg;
ros::Publisher pub_PouseStartFlg;
ros::Publisher pub_PouseStopFlg;
ros::Publisher pub_ConnectStartFlg;
ros::Publisher pub_ConnectStopFlg;
ros::Publisher pub_BatteryStartFlg;
ros::Publisher pub_BatteryStopFlg;
ros::Publisher pub_GpioStartFlg;

ros::Publisher pub_stop_flg;
ros::Publisher pub_ctlvehicleFlg;
ros::Publisher pub_waitstate_flg;

autoware_msgs::WaypointCustom now_wpc;

static const int16_t STOP = 0;
static const int16_t RUN = 1;
static const int16_t OFF = 0;
static const int16_t ON = 1;
static const int16_t ERR = 0;
static const int16_t OK = 1;
static const int16_t INIT = 9;

// 上位サーバの走行/停止指示
int16_t SrvStartFlg = INIT;
int16_t SrvStopFlg = INIT;
// 信号検知の走行/停止指示
int16_t TlrStartFlg = INIT;
int16_t TlrStopFlg = INIT;
// 一旦停止の走行/停止指示
int16_t PouseStartFlg = INIT;
int16_t PouseStopFlg = INIT;
// 上位通信途絶の走行/停止指示
int16_t ConnectStartFlg = INIT;
int16_t ConnectStopFlg = INIT;
// バッテリ交換推奨時の停止、発進指示
int16_t BatteryStartFlg = INIT;
int16_t BatteryStopFlg = INIT;
// 強制発進
int16_t GpioStartFlg = INIT;
int16_t GFWaypointId = 0;
// 障害発生時に障害通知機能からの停止用
int16_t MStatusFlg = INIT;
// ルート切り替え中の停止用
int16_t RCStatusFlg = INIT;
int16_t AutowareFlg = INIT;
// 運転準備ボタン
int16_t DriveBotton = INIT;
// 自動運転切り替えスイッチ
int16_t AutoDriveSW = INIT;
int16_t TagnovaFlg = INIT;
int16_t SystemFlg = INIT;

// 実車速
float TagnovaSpeed = 0.0;

int16_t RunFlg = INIT;
int16_t StopFlg = INIT;
int16_t WaypointId = 0;
int16_t IsRun = INIT;

// VehicleCtlState
static const int16_t NOW = 0;
int16_t VehicleStatus = STOP;
bool DisplayFlg = false;

void ShowDisplayLog(std::string str)
{
  char date[64];
  time_t t = time(NULL);
  strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
  printf("%s %s\n",date,str.c_str());
  
}

void PublishVehicleState(int16_t RunFlg_,int16_t SrvStartFlg_,int16_t TlrStartFlg_,int16_t PouseStartFlg_,int16_t ConnectStartFlg_,int16_t GpioStartFlg_)
{
  /*
    待ち状態をパブリッシュ
    待ちなし：0
    上位待ち：1（server_0_0）
    車両発進ボタン待ち：2（未対応、自動運転開始時のみ表示）
    信号待ち：3（traffic_signal_0_0）
    端末待ち：4（廃止）
    一旦停止待ち：5
    上位通信途絶：6
  */ 
  std_msgs::Int16 msg;
  msg.data = 0;

  // 走行中
  if (RunFlg_ == RUN)
  {
    msg.data = 0;
  }
  else
  {
    // 自動運転開始時に発進ボタン待ちを表示
    if (GpioStartFlg_ == INIT)
    {
      msg.data = 2;
    }
    // 停止箇所停止中に上位待ちを表示
    else if (SrvStartFlg_ == STOP)
    {
      msg.data = 1;
    }
    // 信号が赤の場合に信号待ちを表示
    else if (TlrStartFlg_ == STOP)
    {
      msg.data = 3;
    }
    // 一旦定時に一旦停止待ちを表示
    else if (PouseStartFlg_ == STOP)
    {
      msg.data = 5;
    }
  }

  pub_waitstate_flg.publish(msg);
  //ros::spinOnce();
  return;  
}

// 強制発進フラグの送信
void publishGpioStartFlg(const int flg) {
  std::string displayMsg = "Tagnova Start botton ";
  ShowDisplayLog(displayMsg + (flg == ON ? "On" : flg == OFF ? "Off" : "Init") + ".");
  std_msgs::Int16 msg;
  msg.data = GpioStartFlg = flg;
  pub_GpioStartFlg.publish(msg);
}

// 発進指示ON
int16_t RunOrderStatus(int16_t SFlg_,int16_t TFlg_,int16_t PFlg_,int16_t CFlg_,int16_t BFlg_,int16_t GFlg_,const int16_t WaypointId_)
{
  std_msgs::Int16 on_msg;
  on_msg.data = ON;
  std_msgs::Int16 off_msg;
  off_msg.data = OFF;

  // 走行指示判定
  // 機台車両ボタンによる強制走行
  if (GFlg_ == ON)
  {
    if (RunFlg != ON)
    {
      GFWaypointId = WaypointId_; // 機台車両ボタンを推したWaypointを保持
      //printf("GFWaypointId:%d,WaypointId_:%d\n",GFWaypointId,WaypointId_);
      ROS_INFO("DEBUG: RunOrderStatus: Running status force on.");
      if (SFlg_ != ON)
      {
        pub_SrvStartFlg.publish(on_msg);
        SrvStartFlg = ON;
      }
      if (TFlg_ != ON)
      {
        pub_TlrStartFlg.publish(on_msg);
        TlrStartFlg = ON;
      }
      if (PFlg_ != ON)
      {
        pub_PouseStartFlg.publish(on_msg);
        PouseStartFlg = ON;
      }
      if (CFlg_ != ON)
      {
        pub_ConnectStartFlg.publish(on_msg);
        ConnectStartFlg = ON;
      }
      // BatteryStartFlgは強制発進無効であるため、ON送信は行わないがGpioStartFlgのOff送信は行う
      // 現状が強制発進状態なので、一旦解除しないとBatteryStopFlgのOff送信を行うタイミングを作れず、バッテリ警告後の発進できないため
      if (BFlg_ != ON) {
        publishGpioStartFlg(OFF);
      }
    }
    else
    {
      // 次のWaypointに到達したら機台車両ボタンの状態をOFFにする
      if (GFWaypointId != WaypointId_)
      {
        publishGpioStartFlg(OFF);
      }
    }
    
    //ros::spinOnce();
    // 発進指示フラグON
    return BFlg_ == OFF ? OFF : ON; // BatteryStartFlgがOFFだった場合は発進させない。OFF以外ならば発進する。
  }
  else if (SFlg_ == ON && TFlg_ == ON && PFlg_ == ON && CFlg_ == ON && BFlg_ == ON && GFlg_ != INIT)
  {
    if (RunFlg == OFF)
    {
      ROS_INFO("DEBUG: RunOrderStatus: Running status on.");
    }
    // 発進指示フラグON
    return ON;
  }
  else
  {
    // 発進指示なし
    // ROS_DEBUG("DEBUG: RunOrderStatus: Running status none.");
    // 発進指示フラグ操作なし
    return OFF;
  }

}

// 停止指示ON
int16_t StopOrderStatus(int16_t SFlg_,int16_t TFlg_,int16_t PFlg_,int16_t CFlg_,int16_t BFlg_,int16_t GFlg_)
{
  std_msgs::Int16 off_msg;
  off_msg.data = OFF;

  // 停止指示判定
  if (GFlg_ == ON)
  {
    // ROS_INFO("DEBUG: StopOrderStatus: Stopping status force off.");
    pub_SrvStopFlg.publish(off_msg);
    pub_TlrStopFlg.publish(off_msg);
    pub_PouseStopFlg.publish(off_msg);
    pub_ConnectStopFlg.publish(off_msg);
    // BatteryStopFlgは強制発進無効であるため、送信しない
    return BFlg_ == ON ? ON : OFF; // BatteryStopFlgがONだった場合は発進させない。OFF以外ならば発進する。
  }
  else if ((SFlg_ == ON || TFlg_ == ON || PFlg_ == ON || CFlg_ == ON || BFlg_== ON) && (GFlg_ == OFF))
  {
    ROS_INFO("DEBUG: StopOrderStatus: Stopping status on.");
    // 各走行TopicフラグOFF
    if (SFlg_ == ON)
    {
      pub_SrvStartFlg.publish(off_msg);
      pub_SrvStopFlg.publish(off_msg);
      SrvStartFlg = OFF;
    }
    if (TFlg_ == ON)
    {
      pub_TlrStartFlg.publish(off_msg);
      pub_TlrStopFlg.publish(off_msg);
      TlrStartFlg = OFF;
    }
    if (PFlg_ == ON)
    {
      pub_PouseStartFlg.publish(off_msg);
      pub_PouseStopFlg.publish(off_msg);
      PouseStartFlg = OFF;
    }
    if (CFlg_ == ON)
    {
      pub_ConnectStartFlg.publish(off_msg);
      pub_ConnectStopFlg.publish(off_msg);
      ConnectStartFlg = OFF;
    }
    if (BFlg_ == ON)
    {
      pub_BatteryStartFlg.publish(off_msg);
      pub_BatteryStopFlg.publish(off_msg);
      BatteryStartFlg = OFF;
    }
    //ros::spinOnce();
    // 停止指示フラグON
    return ON;
  }
  else
  {
    // 発進指示なし
    // ROS_DEBUG("DEBUG: RunOrderStatus: Stopping status none.");
    // 停止指示フラグ操作なし
    return OFF;
  }

}

// 自動運転システム状態ON/OFF
int16_t AutowareStatus(int16_t MSFlg_,int16_t RCFlg_)
{
  // 自動運転システム判定
  if (MSFlg_ == OK && RCFlg_ == OK)
  {
    // Autoware状態正常
    // ROS_DEBUG("DEBUG: AutowareStatus: Autoware status Healthy.");
    return OK;
  }
  else
  {
    // Autoware障害の時
    /* 
    if (MSFlg_ == NG)
    {
      // 障害時走行停止する。機台車両ボタンをOFFにする。
      PublishGPIOStateCmd(OFF); 
    }
    */
    // Autoware状態異常
    // ROS_INFO("DEBUG: AutowareStatus: Autoware status Unhealthy.");
    return ERR;
  }

}

// 車両状態
int16_t TagnovaStatus(int16_t DBFlg_,int16_t ADSFlg_)
{
  // 車両状態判定
  if (DBFlg_ == OK && ADSFlg_ == OK)
  {
    // タグノバ状態正常
    // ROS_DEBUG("DEBUG: TagnovaStatus: Tagnova status Healthy.");
    return OK;
  }
  else
  {
    // タグノバ状態異常
    // ROS_INFO("DEBUG: TagnovaStatus: Tagnova status Unhealthy.");
    // 手動運転に切り替わったと判断し、機台車両ボタンを初期化する。
    if (GpioStartFlg != INIT)
    {
      publishGpioStartFlg(INIT);
    }
    return ERR;
  }

}

// 自動運転システム状態
int16_t SystemStatus(int16_t MSFlg_,int16_t RCFlg_,int16_t DBFlg_,int16_t ADSFlg_)
{
  int16_t AwFlg_;
  int16_t TgFlg_;
  // Autoware状態
  AwFlg_ = AutowareStatus(MSFlg_,RCFlg_);
  // 車両状態
  TgFlg_ = TagnovaStatus(DBFlg_,ADSFlg_);

  // 自動運転システム状態判定
  if (AwFlg_ == OK && TgFlg_ == OK)
  {
    // タグノバ状態正常
    // ROS_DEBUG("DEBUG: SystemStatus: System status Healthy.");
    return OK;
  }
  else
  {
    // タグノバ状態異常
    // ROS_INFO("DEBUG: SystemStatus: System status Unhealthy.");
    return ERR;
  }
  
}

// 走行状態
int16_t RunningStatus(float tagval_)
{
  //printf("Speed:[%2.1lf]\n", tagval_);
  if (tagval_ > 0.0)
  {
    return ON;
  }
  else
  {
    return OFF;
  }

}

void PublishStateCmd(int16_t order_)
{
  std_msgs::Int16 msg;
  msg.data = order_;
  pub_stop_flg.publish(msg);
  //ros::spinOnce();

  return;
}

// 上位連携（システム状態および停止指示がないことを上位指示の前提とする）
void PublishbeforeSrvCmd(int16_t order_)
{
  std_msgs::Int16 msg;
  msg.data = order_;
  pub_ctlvehicleFlg.publish(msg);
  //ros::spinOnce();

  return;
}

void ctlvehicle(const int16_t WaypointId_)
{
  char date[64];
  int16_t RunFlg_ = INIT;
  int16_t StopFlg_ = INIT;
  int16_t SystemFlg_ = INIT;
  int16_t IsRun_ = STOP;
  // 停止指示
  StopFlg_ = StopOrderStatus(SrvStopFlg,TlrStopFlg,PouseStopFlg,ConnectStopFlg,BatteryStopFlg,GpioStartFlg);
  // 走行指示
  RunFlg_ = RunOrderStatus(SrvStartFlg,TlrStartFlg,PouseStartFlg,ConnectStartFlg,BatteryStartFlg,GpioStartFlg,WaypointId_);
  SystemFlg_ = SystemStatus(MStatusFlg,RCStatusFlg,DriveBotton,AutoDriveSW);
  IsRun_ = RunningStatus(TagnovaSpeed);

  // 車両制御
  if (SystemFlg_ == OK)
  {
    // 停止指令（停止位置到着）
    if (StopFlg_ == ON)
    {
      ROS_DEBUG("DEBUG: ctlvehicle: Autoware Running stop.");
      PublishStateCmd(STOP);
      // 上位連携（システム状態および停止指示がないことを上位指示の前提とする）
      PublishbeforeSrvCmd(OFF);
    }
    // 走行指令
    else if (RunFlg_ == ON && StopFlg == OFF)
    {
      ROS_DEBUG("DEBUG: ctlvehicle: Autoware Running start.");
      PublishStateCmd(RUN);      
    }
    else if (StopFlg_ == OFF && RunFlg_ == OFF)
    {
      //ROS_DEBUG("DEBUG: ctlvehicle: Autoware keeping status.");
      PublishbeforeSrvCmd(ON);
    }
    else
    {
      // 停止指令
      ROS_DEBUG("DEBUG: ctlvehicle: Autoware abnomal status.");
      PublishStateCmd(STOP);
      // 上位連携（システム状態および停止指示がないことを上位指示の前提とする）
      PublishbeforeSrvCmd(OFF);
    }

  }  
  else
  {
    // 停止指令（異常時）
    ROS_DEBUG("DEBUG: ctlvehicle: Autoware abnormal stop.");
    PublishStateCmd(STOP);
    // 上位連携（システム状態および停止指示がないことを上位指示の前提とする）
    PublishbeforeSrvCmd(OFF);
  }

  // 端末通知
  PublishVehicleState(RunFlg_,SrvStartFlg,TlrStartFlg,PouseStartFlg,ConnectStartFlg,GpioStartFlg);           

  // Debug
  if (RunFlg_ != RunFlg || StopFlg_ != StopFlg || SystemFlg_ != SystemFlg 
   || WaypointId_ != WaypointId || IsRun_ != IsRun)
  {
    time_t t = time(NULL);
    strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
    printf("%s",date);
    printf(" WaypointID:%d, RunState:%d\n", WaypointId_, IsRun_);
    printf("%s",date);
    printf("    RunFlg:%d (SRF:%d, TRF:%d, PRF:%d, CRF:%d, BRF:%d, GRF:%d)\n", RunFlg_, SrvStartFlg, TlrStartFlg, PouseStartFlg, ConnectStartFlg, BatteryStartFlg, GpioStartFlg);
    printf("%s",date);
    printf("   StopFlg:%d (SSF:%d, TSF:%d, PSF:%d, CSF:%d, BSF:%d) \n", StopFlg_, SrvStopFlg, TlrStopFlg, PouseStopFlg, ConnectStopFlg, BatteryStopFlg);
    printf("%s",date);
    printf(" SystemFlg:%d (MSF:%d, RCF:%d, DBF:%d, ADF:%d)\n", SystemFlg_, MStatusFlg, RCStatusFlg, DriveBotton,AutoDriveSW);

    WaypointId = WaypointId_;
    RunFlg = RunFlg_;
    StopFlg = StopFlg_;
    SystemFlg = SystemFlg_;
    IsRun = IsRun_;
  }
}

void SrvStartCallBack(const std_msgs::Int16 &flg)
{
  if (SrvStartFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Server start flg On.");
  }
  if (flg.data == ON) {
    SrvStartFlg = ON;
  }
}
void SrvStopCallBack(const std_msgs::Int16 &flg)
{
  if (SrvStopFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Server stop flg On.");
  }
  SrvStopFlg = flg.data == ON ? ON : OFF;
}
void TlrStartCallBack(const std_msgs::Int16 &flg)
{
  if (TlrStartFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Traffic Light start flg On.");
  }
  if (flg.data == ON) {
    TlrStartFlg = ON;
  }
}
void TlrStopCallBack(const std_msgs::Int16 &flg)
{
  if (TlrStopFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Traffic Light stop flg On.");
  }
  TlrStopFlg = flg.data == ON ? ON : OFF;
}
void PouseStartCallBack(const std_msgs::Int16 &flg)
{
  if (PouseStartFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Pouse start flg On.");
  }
  if (flg.data == ON) {
    PouseStartFlg = ON;
  }
}
void PouseStopCallBack(const std_msgs::Int16 &flg)
{
  if (PouseStopFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Pouse stop flg On.");
  }
  PouseStopFlg = flg.data == ON ? ON : OFF;
}

void ConnectStartCallBack(const std_msgs::Int16 &flg)
{
  if (ConnectStartFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Connect start flg On.");
  }
  if (flg.data == ON) {
    ConnectStartFlg = ON;
  }
}
void ConnectStopCallBack(const std_msgs::Int16 &flg)
{
  if (ConnectStopFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Connect stop flg On.");
  }
  ConnectStopFlg = flg.data == ON ? ON : OFF;
}

void BatteryStartCallBack(const std_msgs::Int16 &flg)
{
  if (BatteryStartFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Battery start flg On.");
  }
  if (flg.data == ON) {
    BatteryStartFlg = ON;
  }
}
void BatteryStopCallBack(const std_msgs::Int16 &flg)
{
  if (BatteryStopFlg != flg.data && flg.data == ON)
  {
    ShowDisplayLog("Battery stop flg On.");
  }
  BatteryStopFlg = flg.data == ON ? ON : OFF;
}

void GpioStartCallBack(const std_msgs::Int16 &flg)
{
  if (flg.data == ON)
  {
    ShowDisplayLog("Tagnova Start botton On.");
    GpioStartFlg = ON;
  }
}
void MSCheckCallBack(const std_msgs::Int16 &flg)
{
  if (MStatusFlg != flg.data)
  {
    if (flg.data == ERR)
    {
      ShowDisplayLog("Autoware Status Unhealthy.");
    }
    else if (flg.data == OK)
    {
      ShowDisplayLog("Autoware Status Healthy.");
    }
  }
  MStatusFlg = flg.data;
}
void RouteChangeCallBack(const std_msgs::Int16 &flg)
{
  if (RCStatusFlg != flg.data)
  {
    if (flg.data == OFF)
    {
      ShowDisplayLog("Route file Change start.");
    }
    else if (flg.data == ON)
    {
      ShowDisplayLog("Route file Change end.");
    }
  }
  RCStatusFlg = flg.data;
}

void PlcsensorCallback(const udp_msgs::UdpSensorPacket &UdpSensorPacket)
{

  if (DriveBotton != UdpSensorPacket.SOC)
  {
    if (UdpSensorPacket.SOC == OFF)
    {
      ShowDisplayLog("Drive ready Botton Off.");
    }
    else if (UdpSensorPacket.SOC == ON)
    {
      ShowDisplayLog("Drive ready Botton On.");
    }
  }
  if (AutoDriveSW != UdpSensorPacket.ECUMode)
  {
    if (UdpSensorPacket.ECUMode == OFF)
    {
      ShowDisplayLog("Auto Drive Botton Off.");
    }
    else if (UdpSensorPacket.ECUMode == ON)
    {
      ShowDisplayLog("Auto Drive Botton On.");
    }
  }

  DriveBotton = UdpSensorPacket.SOC;
  AutoDriveSW = UdpSensorPacket.ECUMode;  
  TagnovaSpeed = UdpSensorPacket.PLC_vel;

}

void WaypointsCallback(const autoware_msgs::Lane &lane)
{
  now_wpc = lane.waypoints.at(NOW).wpc;
  // 走行制御
  ctlvehicle(now_wpc.waypoint_id); 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ctlvehicle");
  ros::NodeHandle n;
  // 上位サーバの走行/停止指示
  ros::Subscriber sub_SrvStart = n.subscribe("SrvStartFlg", 10, SrvStartCallBack);
  ros::Subscriber sub_SrvStop = n.subscribe("SrvStopFlg", 10, SrvStopCallBack);
  pub_SrvStartFlg = n.advertise<std_msgs::Int16>("SrvStartFlg", 10);
  pub_SrvStopFlg = n.advertise<std_msgs::Int16>("SrvStopFlg", 10);
  // 信号検知の走行/停止指示
  ros::Subscriber sub_TlrStart = n.subscribe("TlrStartFlg", 10, TlrStartCallBack);
  ros::Subscriber sub_TlrStop = n.subscribe("TlrStopFlg", 10, TlrStopCallBack);
  pub_TlrStartFlg = n.advertise<std_msgs::Int16>("TlrStartFlg", 10);
  pub_TlrStopFlg = n.advertise<std_msgs::Int16>("TlrStopFlg", 10);
  //一旦停止の走行/停止指示
  ros::Subscriber sub_PouseStart = n.subscribe("PouseStartFlg", 10, PouseStartCallBack);
  ros::Subscriber sub_PouseStop = n.subscribe("PouseStopFlg", 10, PouseStopCallBack);
  pub_PouseStartFlg = n.advertise<std_msgs::Int16>("PouseStartFlg", 10);
  pub_PouseStopFlg = n.advertise<std_msgs::Int16>("PouseStopFlg", 10);
  //一旦停止の走行/停止指示
  ros::Subscriber sub_ConnectStart = n.subscribe("ConnectStartFlg", 10, ConnectStartCallBack);
  ros::Subscriber sub_ConnectStop = n.subscribe("ConnectStopFlg", 10, ConnectStopCallBack);
  pub_ConnectStartFlg = n.advertise<std_msgs::Int16>("ConnectStartFlg", 10);
  pub_ConnectStopFlg = n.advertise<std_msgs::Int16>("ConnectStopFlg", 10);
  // バッテリ交換推奨時の停止/発進
  ros::Subscriber sub_BatteryStart = n.subscribe("BatteryStartFlg", 10, BatteryStartCallBack);
  ros::Subscriber sub_BatteryStop = n.subscribe("BatteryStopFlg", 10, BatteryStopCallBack);
  pub_BatteryStartFlg = n.advertise<std_msgs::Int16>("BatteryStartFlg", 10);
  pub_BatteryStopFlg = n.advertise<std_msgs::Int16>("BatteryStopFlg", 10);
  //車両機台スイッチからの走行指示
  ros::Subscriber sub_GpioStart = n.subscribe("GpioStartFlg", 10, GpioStartCallBack);
  pub_GpioStartFlg = n.advertise<std_msgs::Int16>("GpioStartFlg", 10);

  //障害発生時に障害通知機能からの停止用
  ros::Subscriber sub_Msc = n.subscribe("MsCheckFlg", 10, MSCheckCallBack);
  //ルート切り替え中の停止用
  ros::Subscriber sub_RC = n.subscribe("RouteChangeCheckFlg", 10, RouteChangeCallBack);

  // PLC（タグノバ）
  ros::Subscriber sub_plcsensor = n.subscribe("plc_sensor_packet", 10, PlcsensorCallback);
  ros::Subscriber sub_waypoints = n.subscribe("safety_waypoints", 10, WaypointsCallback);

  // ros::Rate loop_rate(10);
  pub_stop_flg = n.advertise<std_msgs::Int16>("stop_flg", 10);
  pub_ctlvehicleFlg = n.advertise<std_msgs::Int16>("ctlvehicle_flg", 10);
  // 待ち状態通知
  pub_waitstate_flg = n.advertise<std_msgs::Int16>("waitstate_flg", 10);

  ros::spin();
  return 0;
}
