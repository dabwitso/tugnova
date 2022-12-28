#!/usr/bin/env python3

import rospy
from gps_common.msg import GPSFix
from sensor_msgs.msg import NavSatFix
from communication_msgs.msg import TugnovaGPS

def callback(msg):

    gpsMessage.header.frame_id = "gps"
    gpsMessage.header.stamp = rospy.Time.now()
    gpsMessage.latitude = msg.latitude
    gpsMessage.longitude = msg.longitude
    gpsMessage.speed = 0
    pub.publish(gpsMessage)

rospy.init_node('gnss_preprocessor')

gpsMessage = TugnovaGPS()
sub = rospy.Subscriber('/fix', NavSatFix, callback)
pub = rospy.Publisher('/preprocessed_fix', TugnovaGPS, queue_size=10)

rospy.spin()
