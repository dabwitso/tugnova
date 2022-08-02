#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import yaml
import os
import time
from rosnode import rosnode_ping
from carctl_msgs.msg import monitor_status
from carctl_msgs.msg import emergency_status

pub_pingnode_checker = rospy.Publisher('/monitor_status', monitor_status, queue_size=10)
pub_emergency_error = rospy.Publisher('/emergency_error', emergency_status, queue_size=10)

def rosPub(result):
    INFO = 0   # 正常
    ERROR = 1  # 異常

    msg = monitor_status()
    msg.service_name = "node"
    msg.status = ERROR
    msg.error_msg = "node_1_61"

    emergency_status_msg = emergency_status()
    emergency_status_msg.service_name = "ping_node_checker"
    emergency_status_msg.status = ERROR

    if result:
        msg.status = INFO
        msg.error_msg = "INFO node status ok"
        emergency_status_msg.status = INFO
    else:
        #ping_node停止
        rospy.logerr('ping_node down')
        
    pub_pingnode_checker.publish(msg)
    pub_emergency_error.publish(emergency_status_msg)

def nodeCheck(count):
    result = False
    num = 0
    for num in range(count):
        num += 1
        result = rosnode_ping("ping_node", 1, verbose=False)
        if result:
            break

    rosPub(result)


def yamlRead():
    try:
        yamlpath = os.environ['HOME'] + \
            '/Autoware/ros/src/carctlsystem/packages/monitoring_health/scripts/'
        yamlfile = yamlpath + 'monitoring_health.yaml'
        file = open(yamlfile, 'r')
        yml = yaml.load(file)
        file.close()

        yaml_nodecheck = yml.get("nodecheck")
        post = yml.get("post")
        return yaml_nodecheck, post
    except Exception as e:
        file.close()
        raise

def run():
    rospy.init_node('ping_node_checker', anonymous=True)

    yaml_nodecheck, post = yamlRead()
    timer = yaml_nodecheck.get("timer")
    count = yaml_nodecheck.get("count")

    while not rospy.is_shutdown():
        nodeCheck(count)
        time.sleep(timer)

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(e)
