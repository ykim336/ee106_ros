#!/usr/bin/env python3

import rospy
import roslib
# This might need adjustment based on your actual package name where this script resides
# roslib.load_manifest('ee106s23') 
import sys
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf2_ros
import geometry_msgs.msg

# Constants for the wall following behavior
LINEAR_VELOCITY = 0.2  # meters per second
ANGULAR_VELOCITY = 0.5  # radians per second
WALL_DISTANCE_TARGET = 0.5  # meters - desired distance from the wall
DISTANCE_TOLERANCE = 0.1 # meters - tolerance for maintaining distance
SCAN_ANGLE_RANGE = np.deg2rad(45) # radians - angle range to consider for wall detection on the left

class turtlebot_behavior:

    def __init__(self):

        rospy.loginfo("Initializing TurtleBot Behavior Node...")

        # Initialize Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback)

        # TF Listener for transformations if needed (though the provided structure doesn't strictly require it for basic wall following)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.trans = None
        self.rot = None

        # FSM State and Transition Parameters
        self.state = "forward" # Initial state
        # States can be: "forward", "turn_left", "turn_right"

        # Transition parameters based on LiDAR data
        self.distance_to_wall = float('inf')
        self.extreme_left_detected = False

        self.rate = rospy.Rate(10.0) # 10 Hz

        rospy.loginfo("TurtleBot Behavior Node Initialized.")

        # Main loop for FSM state updates and command sending
        while not rospy.is_shutdown():

            # In this basic implementation, we primarily rely on the callback to update state transition parameters.
            # TF listening is commented out as it's not strictly necessary for this wall following logic
            # try:
            #     self.trans, self.rot = self.tf_buffer.lookup_transform('base_scan', 'left_limit', rospy.Time(0))
            # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #     rospy.logwarn("Could not get transform between 'base_scan' and 'left_limit'")
            #     self.trans = None
            #     self.rot = None


            # Determine the next state based on current state and transition parameters
            next_state = self.state

            if self.state == "forward":
                if self.distance_to_wall < WALL_DISTANCE_TARGET - DISTANCE_TOLERANCE:
                    next_state = "turn_right" # Too close, turn away
                elif self.distance_to_wall > WALL_DISTANCE_TARGET + DISTANCE_TOLERANCE:
                     next_state = "turn_left" # Too far, turn towards
                # If within tolerance, stay in "forward" state

            elif self.state == "turn_left":
                 if self.distance_to_wall >= WALL_DISTANCE_TARGET - DISTANCE_TOLERANCE and self.distance_to_wall <= WALL_DISTANCE_TARGET + DISTANCE_TOLERANCE:
                     next_state = "forward" # Wall is at desired distance, go forward

            elif self.state == "turn_right":
                 if self.distance_to_wall >= WALL_DISTANCE_TARGET - DISTANCE_TOLERANCE and self.distance_to_wall <= WALL_DISTANCE_TARGET + DISTANCE_TOLERANCE:
                     next_state = "forward" # Wall is at desired distance, go forward


            # Update the current state
            if self.state != next_state:
                rospy.loginfo(f"State transition: {self.state} -> {next_state}")
                self.state = next_state

            # Send velocity command based on the current state
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

    # this is the LiDAR callback that will update our FSM transition parameters
    def callback(self, data):

        if data.angle_increment == 0:
            rospy.logwarn("LaserScan angle_increment is zero, cannot process data.")
            return

        # Determine the indices corresponding to the left side angle range
        # We need to find the indices that cover the angle_min to angle_min + SCAN_ANGLE_RANGE
        # considering the angle_increment
        start_angle = data.angle_min
        end_angle = data.angle_min + SCAN_ANGLE_RANGE

        start_idx = int((start_angle - data.angle_min) / data.angle_increment)
        end_idx = int((end_angle - data.angle_min) / data.angle_increment)

        # Ensure indices are within the bounds of the scan data
        start_idx = max(0, start_idx)
        end_idx = min(len(data.ranges), end_idx)

        min_dist_left_side = float('inf')

        # Iterate through the relevant part of the scan data
        for i in range(start_idx, end_idx):
            current_range = data.ranges[i]

            # Check if the measurement is valid (not inf or NaN)
            if not np.isinf(current_range) and not np.isnan(current_range):
                # In this simple implementation, we just take the minimum distance
                # within the left sector. More sophisticated approaches could involve
                # averaging or fitting lines.
                min_dist_left_side = min(min_dist_left_side, current_range)

        # Update the FSM transition parameter
        self.distance_to_wall = min_dist_left_side

        # rospy.loginfo(f"Min distance on left side: {self.distance_to_wall:.2f} m")


    # This function is useful for transforming points if needed, but not strictly
    # necessary for the basic wall following logic based on range data directly.
    def calculate_position_of_range(self, range, idx, angle_increment, angle_min):

        if str(range)=="inf" or np.isnan(range):
            rospy.loginfo("The provided range is infinite or NaN!")
            return None

        theta = idx * angle_increment + angle_min
        x = range * np.cos(theta)
        y = range * np.sin(theta)

        return x,y

def main(args):
    rospy.init_node('left_wall_following', anonymous=True)
    ic = turtlebot_behavior()
    try:
        rospy.spin() # This keeps the node alive, but the main loop is in __init__
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)