#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Header

def talker():
    rospy.init_node('talker')
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
      header = Header()
      header.stamp = rospy.Time.now()

      content = "welcome to the Robotics Lab " + str(header.stamp)
      pub.publish(content)
      rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
