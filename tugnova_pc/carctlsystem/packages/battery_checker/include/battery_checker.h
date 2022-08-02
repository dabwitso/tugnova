#ifndef BATTERY_CHECKER_H
#define BATTERY_CHECKER_H

static const int BATTERY_OK = 0; // バッテリ正常 Normal
static const int BATTERY_LOW = 1; // 交換推奨警告 replacement/recharge recommended warning
static const int BATTERY_LOW_STOP = 2; // 交換推奨警告後、車両停止 stop car, recharge required
static const int BATTERY_LOW_RUN = 3; // 交換推奨警告後、走行指示 drive after recharge required warning
static const int BATTERY_LOW_EXCHANGE_CONFIRM = 4; // 交換推奨警告後、交換指示 instruction to recharge
static const int BATTERY_EXCHANGE = 5; // 要交換警告 recharge required warning
static const int BATTERY_EXCHANGE_STOP = 6; // 要交換警告後、車両停止 stop vehicle
static const int BATTERY_FATAL = 7; // 電圧異常 voltage problem, fatal

static const int COLOR_NORMAL = 0; // 正常時の背景色
static const int COLOR_WARNING = 1; // 警告時の背景色
static const int COLOR_ERROR = 2; // エラー時の背景色
static const int COLOR_FATAL = 3; // 電圧異常時の背景色

const int CHECK_AND_STOP = 1; // 判定＆停止位置
const int CHECK_ONLY = 2; // 判定位置
const int STOP_ONLY = 3; // 停止位置

#endif /* BATTERY_CHECKER_H */
