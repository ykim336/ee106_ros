#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(data.data) # Please consider why ".data" gives the content of "String"


def listener():
    rospy.init_node('listener')
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

