#ifndef plc_sensor_packet_callback_h
#define plc_sensor_packet_callback_h

#include "udp_msgs/UdpSensorPacket.h"

// 
// PLC(coms driver) のコールバック処理を共通化する
// グローバル変数を用意して随時参照してもらう
// 実装ロジック及びcallback処理自体はここに隠ぺいする
//

// PLC(comsからの情報)
int plc_status = 0; // int16 ECU_vel
int plc_error_id = 0; // int16 BrakePotVol
float plc_average_vehicle_speed = 0.0;// float32 PLC_vel

// コールバック関数
//  PLCからの情報をSubして参照用の変数へ格納する
void plc_sensor_packet_Callback(const udp_msgs::UdpSensorPacket& coms_data_) {
  plc_status = coms_data_.ECU_vel;
  plc_error_id = coms_data_.BrakePotVol;
  plc_average_vehicle_speed = coms_data_.PLC_vel;
  //// fprintf(stdout, "DEBUG: plc_sensor_packet_Callback: ECU_vel=[%d] BrakePotVol=[%d] PLC_vel[%f]\n",
  //  plc_status, plc_error_id, plc_average_vehicle_speed);
  //fflush(stdout);
}

// 現在時点のPLC情報を取得する
//  i/o : n メイン側で定義したパラメータの参照
void getPlcSensorInfo(int& status_, int& error_id_, float& average_vehicle_speed_) {
  status_ = plc_status;
  error_id_ = plc_error_id;
  average_vehicle_speed_ = plc_average_vehicle_speed;
}

// plc_sensor_packet のコールバック 初期化処理
//  i : n メイン側で定義したノードハンドルの参照
//  return : rosに登録したSubscriberオブジェクト
ros::Subscriber init_plc_sensor_packet_callback(ros::NodeHandle& n) {
  // fprintf(stdout, "DEBUG: init_plc_sensor_packet_callback: 初期化実行\n");
  return n.subscribe("plc_sensor_packet", 10, plc_sensor_packet_Callback);
}

#endif // plc_sensor_packet_callback_h
