#!/usr/bin/env python3
import rospy
import tf
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String   # ← fix import here

class RangeChecker:
    def __init__(self):
        rospy.init_node('ranges_check')
        self.sub = rospy.Subscriber('/front/scan', LaserScan, self.cb)
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher('/jackal_robot_status', String, queue_size=1)

    def cb(self, data):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('front_bumper', 'front_laser', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed, skipping this scan")
            return

        # Build 4×4 transform matrix
        tf_mat = tf.transformations.quaternion_matrix(rot)
        tf_mat[0:3,3] = trans

        worst = float('inf')
        for i, r in enumerate(data.ranges):
            if math.isinf(r):
                continue
            angle = data.angle_min + i * data.angle_increment
            x_l = r * math.cos(angle)
            y_l = r * math.sin(angle)
            p = np.array([x_l, y_l, 0, 1.0])
            p_mapped = tf_mat.dot(p)
            dist = math.hypot(p_mapped[0], p_mapped[1])
            worst = min(worst, dist)

        if worst == float('inf'):
            status = 'no_data'
        elif worst < 0.2:
            status = 'critical'
        elif worst < 0.5:
            status = 'major'
        else:
            status = 'minor'

        rospy.loginfo(f"Status: {status}")      # debug log
        self.pub.publish(String(data=status))

if __name__ == '__main__':
    rc = RangeChecker()
    rospy.spin()

