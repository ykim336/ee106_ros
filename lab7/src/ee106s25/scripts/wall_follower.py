#!/usr/bin/env python3

import rospy
import sys
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf2_ros
import geometry_msgs.msg

LINEAR_VELOCITY = 0.2
ANGULAR_VELOCITY = 0.5
WALL_DISTANCE_THRESHOLD = 0.6

FRONT_ANGLE_RANGE = np.deg2rad(30)
LEFT_ANGLE_START = np.deg2rad(60)
LEFT_ANGLE_END = np.deg2rad(120)


class turtlebot_behavior:

    def __init__(self):

        rospy.loginfo("Initializing TurtleBot Behavior Node (FSM based on Lab Matrix)...")

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.state = "forward"

        self.left_side_occupied = False
        self.front_side_occupied = False

        self.rate = rospy.Rate(10.0)

        rospy.loginfo("TurtleBot Behavior Node Initialized. Starting FSM.")

        while not rospy.is_shutdown():

            next_state = self.state

            if not self.left_side_occupied and not self.front_side_occupied:
                next_state = "turn_left"
            elif not self.left_side_occupied and self.front_side_occupied:
                next_state = "turn_right"
            elif self.left_side_occupied and not self.front_side_occupied:
                next_state = "forward"
            elif self.left_side_occupied and self.front_side_occupied:
                next_state = "turn_right"

            if self.state != next_state:
                rospy.loginfo(f"State transition: {self.state} -> {next_state}")
                self.state = next_state

            cmd_vel_msg = Twist()
            if self.state == "forward":
                cmd_vel_msg.linear.x = LINEAR_VELOCITY
                cmd_vel_msg.angular.z = 0.0
            elif self.state == "turn_left":
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = ANGULAR_VELOCITY
            elif self.state == "turn_right":
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = -ANGULAR_VELOCITY

            self.cmd_vel_pub.publish(cmd_vel_msg)

            self.rate.sleep()

    def callback(self, data):

        if data.angle_increment == 0:
            rospy.logwarn("LaserScan angle_increment is zero, cannot process data.")
            return

        self.left_side_occupied = False
        self.front_side_occupied = False

        front_angle_start = -FRONT_ANGLE_RANGE / 2.0
        front_angle_end = FRONT_ANGLE_RANGE / 2.0

        try:
             center_idx = int((-data.angle_min) / data.angle_increment)
        except ValueError:
             rospy.logwarn("0 degree not in scan range, cannot determine front center index.")
             return

        front_start_idx_rel_center = int(np.deg2rad(-FRONT_ANGLE_RANGE/2.0) / data.angle_increment)
        front_end_idx_rel_center = int(np.deg2rad(FRONT_ANGLE_RANGE/2.0) / data.angle_increment)

        front_start_idx = center_idx + front_start_idx_rel_center
        front_end_idx = center_idx + front_end_idx_rel_center

        front_start_idx = max(0, front_start_idx)
        front_end_idx = min(len(data.ranges) - 1, front_end_idx)

        for i in range(front_start_idx, front_end_idx + 1):
            current_range = data.ranges[i]
            if not np.isinf(current_range) and not np.isnan(current_range) and current_range < WALL_DISTANCE_THRESHOLD:
                self.front_side_occupied = True
                break

        left_start_idx_rel_center = int(np.deg2rad(LEFT_ANGLE_START) / data.angle_increment)
        left_end_idx_rel_center = int(np.deg2rad(LEFT_ANGLE_END) / data.angle_increment)

        left_start_idx = center_idx + left_start_idx_rel_center
        left_end_idx = center_idx + left_end_idx_rel_center

        left_start_idx = max(0, left_start_idx)
        left_end_idx = min(len(data.ranges) - 1, left_end_idx)

        for i in range(left_start_idx, left_end_idx + 1):
            current_range = data.ranges[i]
            if not np.isinf(current_range) and not np.isnan(current_range) and current_range < WALL_DISTANCE_THRESHOLD:
                self.left_side_occupied = True
                break

    def calculate_position_of_range(self, range, idx, angle_increment, angle_min):

        if str(range)=="inf" or np.isnan(range):
            rospy.loginfo("The provided range is infinite or NaN!")
            return None

        theta = idx * angle_increment + angle_min
        x = range * np.cos(theta)
        y = range * np.sin(theta)

        return x,y


def main(args):
    rospy.init_node('left_wall_following_fsm', anonymous=True)
    ic = turtlebot_behavior()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)