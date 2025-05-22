#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin
import numpy as np
import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Turtlebot():

    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        try:
            self.run(4.0, 0.0)
            self.run(4.0, 4.0)
            self.run(0.0, 4.0)
            self.run(0.0, 0.0)
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self, desired_x, desired_y):
        desired_theta = atan2(desired_y - self.pose.y, desired_x - self.pose.x)
        turn_ctr = Controller(1.5, 0.2, set_point=desired_theta)
        vel = Twist()
        while not rospy.is_shutdown():
            heading_error = desired_theta - self.pose.theta
            while heading_error > pi:
                heading_error -= 2 * pi
            while heading_error < -pi:
                heading_error += 2 * pi

            if abs(heading_error) < 0.01:
                break

            vel.linear.x = 0.0
            vel.angular.z = turn_ctr.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        for _ in range(10):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        drive_ctr = Controller(1.5, 0.2, set_point=desired_theta)

        while not rospy.is_shutdown():
            dx = desired_x - self.pose.x
            dy = desired_y - self.pose.y
            distance = sqrt(dx**2 + dy**2)

            if distance < 0.05:
                break

            desired_theta = atan2(dy, dx)
            drive_ctr.set_point = desired_theta

            heading_error = desired_theta - self.pose.theta
            while heading_error > pi:
                heading_error -= 2 * pi
            while heading_error < -pi:
                heading_error += 2 * pi

            vel.linear.x = 0.3
            vel.angular.z = drive_ctr.update(self.pose.theta) if abs(heading_error) > 0.01 else 0.0

            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        for _ in range(10):
            self.vel_pub.publish(vel)
            self.rate.sleep()
            
    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            # display (x, y, theta) on the terminal
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        error = self.set_point - current_value
        while error > pi:
            error -= 2 * pi
        while error < -pi:
            error += 2 * pi
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

if __name__ == '__main__':
    whatever = Turtlebot()
