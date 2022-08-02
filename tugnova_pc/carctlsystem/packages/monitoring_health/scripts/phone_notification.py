#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import logging
import os
import csv
import requests
import time
import yaml
import json
from requests.packages.urllib3.util.retry import Retry
from requests.adapters import HTTPAdapter

from std_msgs.msg import String
from std_msgs.msg import Int16
from udp_msgs.msg import UdpSensorPacket
from carctl_msgs.msg import phone_msg
from autoware_msgs.msg import Lane
from visualization_msgs.msg import MarkerArray

# 変数定義
carstatus = "non-working"
selfLocalization = "acquired"
waypointId = 0
waypointId_ = 0
messageIdList = {}

NORMAL = 0
ERROR = 1
DETECTION_ERROR = 2



class monitor_status:
    messageId = None
    selfLocalization = None

def yamlRead():
    """ YAMLファイルを読み込む """
    try:
        yamlpath = os.environ['HOME'] + \
            '/Autoware/ros/src/carctlsystem/packages/monitoring_health/scripts/'
        yamlfile = yamlpath + 'monitoring_health.yaml'
        file = open(yamlfile, 'r')
        yml = yaml.load(file)
        file.close()
        post = yml.get("post")
    except Exception as e:
        print(e)
        file.close()
    return post


def fileWrite(monitor_status_, carstatus_, mode_):
    """ VEHICLE_STATSにステータスを書き込む """
    selfLocalization_ = monitor_status_.selfLocalization
    messageId_ = monitor_status_.messageId
    # messageIdがなければ終了
    if messageId_ is None or messageId_ == "":
        print('DEBUG: fileWrite: messageId: None')
        return

    # ファイル出力
    # ファイルパス
    apipath = os.environ['HOME'] + '/Autoware/api/'
    csvfile = apipath + 'VEHICLE_STATUS'
        
    # POSTとはmessageIdが異なるため変換
    # 発進可能
    if monitor_status_.messageId == "status_0_0":
        messageId_ = "api_0_2"
    # 発進中
    elif monitor_status_.messageId == "status_0_1":
        messageId_ = "api_0_3"

    try:
        print('DEBUG: fileWrite: 状態書き込み:VEHICLE_STATUS')
        rospy.loginfo('fileWrite: MassageID: {0}、VehicleStatus: {1}、SelfLocalization: {2} Mode: {3}'.format(
        messageId_, carstatus_, selfLocalization_, mode_))
        with open(csvfile, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([carstatus_, selfLocalization_, messageId_, mode_])
    except Exception as e:
        print(e)
        print('DEBUG: fileWrite: 状態書き込みエラー:VEHICLE_STATUS')

    return


def phoneNotification(monitor_status_, carstatus_):
    """ 端末通知 """
    # パラメタ初期値
    url = "http://10.42.0.10:8080/v/status"
    timeout = 1.0
    max_times = 3
    mode = "init"
    lane = ""

    # YAMLファイルからのパラメタ取得
    try:
        url = reqd.get("url")         # 上位サーバのURL
        timeout = reqd.get("timeout")  # 接続タイムアウト値(秒)
        max_times = reqd.get("max_times")  # 試行回数
        mode = reqd.get("mode")  # 起動モード
        lane = rospy.get_param("current_lane_id")
    except:
        print('DEBUG: phoneNotification: YAMLファイル読み取りエラー')

    # monitoring_healthからのSubscribeしたmessageIdをセット
    messageId_ = monitor_status_.messageId
    selfLocalization_ = monitor_status_.selfLocalization

    # messageIdがなければ終了
    if messageId_ is None or messageId_ == "":
        #print('DEBUG: fileWrite: messageId: None')
        return
    # デバッグ
    print('DEBUG: phoneNotification: 状態通知')
    #print('DEBUG: phoneNotification:メッセージID：{0}、車両状態：{1}、自己位置：{2}'.format(
    #    messageId_, carstatus_, selfLocalization_))
    rospy.loginfo('POST: MassageID: {0}、VehicleStatus: {1}、SelfLocalization: {2} Mode: {3} WaypointId: {4} CurrentLane: {5}'.format(
        messageId_, carstatus_, selfLocalization_, mode, waypointId, lane))

    # Getパラメタ
    headers = {'content-type': 'application/json'}
    params = {"messageId": messageId_,
              "status": carstatus_,
              "selfLocalization": selfLocalization_,
              "mode": mode,
              "waypointId": waypointId,
              "currentLane": lane}
    # print(params)  # Debug
    session = requests.Session()
    # リトライ回数 : total
    # sleep時間 : backoff_factor
    # 例外エラーの表示 : raise_for_status
    # status_forcelist : timeout以外でリトライするステータスコード
    retries = Retry(total=max_times,
                    backoff_factor=1,
                    status_forcelist=[403, 404, 500, 502, 503, 504])
    # セッション
    session.mount("http://", HTTPAdapter(max_retries=retries))
    # POST
    # connect timeoutをtimeoutの設定値, read timeoutを2秒に設定
    result = 0
    try:
        response = session.request("POST", url, data=json.dumps(
            params), headers=headers, stream=True, timeout=(timeout, 1.0))
        result = response.status_code
    except requests.exceptions.ConnectTimeout:
        # print('DEBUG: phoneNotification:接続エラー')
        rospy.logerr('Phone connect timeout')
        print('DEBUG: phoneNotification: Phone connect timeout')
    except requests.exceptions.ConnectionError:
        rospy.logerr('Phone connect error')
        print('DEBUG: phoneNotification: Phone connect error')
    else:
        # print('DEBUG: phoneNotification:接続成功')
        rospy.loginfo('Phone connect success')
        # レスポンスをデコード
        # res = response.json()
        # print(response.status_code)    # HTTPのステータスコード取得
        # print(response.text)           # レスポンスのHTMLを文字列で取得
        # print(res)

    return result


def plcsenserCallback(plcmsg):
    """ plc_senser_packet callback """
    global carstatus
    # PLCからの平均車速で走行中/走行可を判断
    if plcmsg.PLC_vel > 0:
        carstatus = "running"
    else:
        carstatus = "idling"

def Phmsgcallback(monitor_status_):
    """ phone_msg callback """
    global monitor_status
    global messageIdList
    global carstatus
    global waypointId_

    current_time = time.time()
    # print('DEBUG: Phmsgcallback:current_time:{0}'.format(current_time))
    waittime = reqd.get("waittime")
    messageId_ = monitor_status_.messageId

    # PLC Collback起因で処理する際に利用
    monitor_status = monitor_status_
    # Waypointが更新されたら通知する
    if waypointId_ != waypointId:
        messageIdList = {}

    # 初回通知実施
    if messageIdList.get(messageId_) == None:
        # 端末へのPOST
        phoneNotification(monitor_status_, carstatus)
        pub_error_state.publish(messageId_)
        # ファイル書き込み
        fileWrite(monitor_status_, carstatus, reqd.get("mode"))
        messageIdList.update([(messageId_, current_time)])
    else:
        elapsed_time = current_time - messageIdList.get(messageId_)
        # 連続したポップアップ通知を抑止
        if elapsed_time > waittime:
            # 端末へのPOST
            phoneNotification(monitor_status_, carstatus)
            pub_error_state.publish(messageId_)
            # ファイル書き込み
            fileWrite(monitor_status_, carstatus, reqd.get("mode"))
            # 該当messageIdの通知した時刻を格納
            messageIdList.update([(messageId_, current_time)])
        else:
            # 通知しない
            pass
    waypointId_ = waypointId

def waypointsCallback(lane):
    # 自動走行モード時のWaypointID取得
    global waypointId
    waypointId = lane.waypoints[0].wpc.waypoint_id

def waypointMarkerCallback(msg):
    # 経路作成モード時のWaypointID取得
    global waypointId
    waypointId = msg.markers[-1].id + 1
    Phmsgcallback(monitor_status)


def run():
    """ rosが起動中は処理を繰り返す """
    global reqd
    global pub_error_state

    try:
        reqd = yamlRead()       # YAMLファイル読み込み
        rospy.init_node('phone_notification', anonymous=True)
        pub_error_state = rospy.Publisher('/send_error_status', String, queue_size=10)

        print('Subscribe開始')
        rospy.Subscriber('/phone_msg', phone_msg, Phmsgcallback)
        rospy.Subscriber('/plc_sensor_packet',
                         UdpSensorPacket, plcsenserCallback)
        rospy.Subscriber('/safety_waypoints', Lane, waypointsCallback)
        rospy.Subscriber('/waypoint_saver_marker', MarkerArray, waypointMarkerCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        logging.error('Phone notification error')


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        logging.error('Subscribe end.')

        # Autowareダウン時通知
        logging.error('Autoware stop notification')

        carstatus = "non-working"
        selfLocalization = "init"
        messageId = "status_1_0"
        mode = "init"
        monitor_status.messageId = messageId
        monitor_status.selfLocalization = selfLocalization

        # monitor_status.messageId = "status_1_0"
        # phoneNotification(monitor_status, carstatus)
        pub_error_state.publish(messageId)
        fileWrite(monitor_status, carstatus, mode)

        pass
