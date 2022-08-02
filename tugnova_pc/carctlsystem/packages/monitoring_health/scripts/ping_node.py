#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import yaml
import os
import time
import json
from rosnode import rosnode_ping
from std_msgs.msg import String
from carctl_msgs.msg import monitor_status
from carctl_msgs.msg import emergency_status
from std_msgs.msg import Int16

NOT_OCCURRED = 1
OCCURRED = 0

scan_respawn_node_list = ['node_1_6','node_1_33','node_1_53','node_1_52']

pub_pingnode = rospy.Publisher('/monitor_status', monitor_status, queue_size=10)
pub_emergency_error = rospy.Publisher('/emergency_error', emergency_status, queue_size=10)

def rosPub(node, status):
    """ 結果を通知(monitor_status=service_name,status,error_msg) """
    global cancel_possible_emergency_error_by_scan
    global cancel_impossible_emergency_error_by_scan
    INFO = 0   # 正常
    ERROR = 1  # 異常
    messageId = None

    msg = monitor_status()
    msg.service_name = "node"

    emergency_status_msg = emergency_status()
    emergency_status_msg.service_name = "node"

    if status:
        msg.status = INFO
        msg.error_msg = "INFO node status ok"
        emergency_status_msg.status = INFO
        #緊急停止が必要なノード停止が発生した場合、緊急停止状態を保持する
        if cancel_possible_emergency_error_by_scan == OCCURRED:
            emergency_status_msg.status = ERROR
        if cancel_impossible_emergency_error_by_scan == OCCURRED:
            emergency_status_msg.status = ERROR

    else:
        msg.status = ERROR
        emergency_status_msg.status = ERROR
        hit = False
        for line in dic['dictionary']:
            if line['nodeName'].find(node) >= 0:
                # 辞書ファイルのメッセージIDを抜き取る
                messageId = line['messageId']
                hit = True
                break

        if hit == False:
            messageId = "node_1_0"
            rospy.logerr('ERROR Node not found in the DICTIONARY')

        msg.error_msg = messageId
        if msg.error_msg in scan_respawn_node_list:
            #自己位置復帰で復帰するノードが停止した場合
            # スキャンでの解除可能な異常発生
            cancel_possible_emergency_error_by_scan = OCCURRED
            rospy.logerr('cancel_possible_emergency_error_by_scan = OCCURRED node:{0}'.format(msg.error_msg))
        else:
            #「自己位置復帰で復帰するノードの停止」以外のノード停止が発生した場合
            # スキャンでの解除不可能状態な異常発生
            cancel_impossible_emergency_error_by_scan = OCCURRED
            rospy.logerr('cancel_impossible_emergency_error_by_scan = OCCURRED node:{0}'.format(msg.error_msg))
             
    pub_pingnode.publish(msg)
    pub_emergency_error.publish(emergency_status_msg)

def nodePing(p_node):
    """ ノードへのrosnode pingの実行(TRUE/FALSE) """
    global count
    count = 3
    result = False
    num = 0
    #rospy.loginfo('最大試行回数：{0}'.format(count))
    for num in range(count):
        num += 1
        result = rosnode_ping(p_node, 1,verbose=False)
        # rospy.loginfo('ノード：{0}の状態：{1}、試行{2}回目'.format(p_node,result,num))
        if result:
            break

    return result


def nodeCheck(nodelist):
    """ pingするノード名をYAMLファイルから取得し処理を繰り返す """
    allnode_result = True
    for node in nodelist:
        result = nodePing(node)
        if not result:
            # ノードが異常な場合
            rospy.logerr('Node: {0} is down'.format(node))
            allnode_result = False
            rosPub(node, result)

    if allnode_result:
        rospy.loginfo('All Node is normal')
        rosPub(node, allnode_result)
    else:
        rospy.logerr('Node is aborting')

    return

def yamlRead():
    """ YAMLファイル読み込み """
    try:
        yamlpath = os.environ['HOME'] + \
            '/Autoware/ros/src/carctlsystem/packages/monitoring_health/scripts/'
        yamlfile = yamlpath + 'monitoring_health.yaml'
        file = open(yamlfile, 'r')
        yml = yaml.load(file)
        file.close()
        yaml_nodecheck = yml.get("nodecheck")
        post = yml.get("post")
    except Exception as e:
        rospy.logerr(e)
        file.close()
    return yaml_nodecheck, post

def dicRead(mode):
    """ 辞書ファイル読み込み """
    try:
        path = '/Autoware/ros/renkei/DICTIONARY/'
        if mode == 'route-creation':
            path = '/Autoware/work/DICTIONARY/'

        dicpath = os.environ['HOME'] + path
        dicfile = dicpath + 'DICTIONARY'
        file = open(dicfile, 'r')
        dictext = file.read()
        file.close()
        dicjson = json.loads(dictext)
    except Exception as e:
        rospy.logerr(e)
        dic.close()
    return dicjson

def scanRespawnResultCallback(msg):
    global cancel_possible_emergency_error_by_scan
    cancel_possible_emergency_error_by_scan = msg.data


def run():
    """ rosが起動中は処理を繰り返す """
    global count
    global dic
    global cancel_possible_emergency_error_by_scan
    global cancel_impossible_emergency_error_by_scan
    cancel_possible_emergency_error_by_scan = NOT_OCCURRED
    cancel_impossible_emergency_error_by_scan = NOT_OCCURRED
    try:
        rospy.init_node('ping_node', anonymous=True)
        yaml_nodecheck, post = yamlRead()
        dic = dicRead(post.get("mode"))
        nodelist = yaml_nodecheck.get("node_name")
        timer = yaml_nodecheck.get("timer")
        count = yaml_nodecheck.get("count")
        
        rospy.Subscriber('/scan_respawn_result', Int16, scanRespawnResultCallback)
        while not rospy.is_shutdown():
            nodeCheck(nodelist)
            time.sleep(timer)
            
    except Exception as e:
        rospy.logerr(e)

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
