#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import pi

LINEAR_SPEED  = 0.1
ANGULAR_SPEED = pi / 12
HZ            = 20
SIDE_LENGTH   = 4.0
TURN_ANGLE    = pi / 2.0

class Turtlebot:
    def __init__(self):
        rospy.init_node("turtlebot_simple_square", anonymous=True)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.sleep(0.5)
        self.rate = rospy.Rate(HZ)
        rospy.loginfo("TurtleBot Simple Square: Node Started. Driving a square.")
        self.run()
        rospy.signal_shutdown("Square complete.")

    def _publish_for_duration(self, twist_msg, duration_seconds):
        ticks = int(duration_seconds * HZ)
        for _ in range(ticks):
            if rospy.is_shutdown():
                return False
            self.vel_pub.publish(twist_msg)
            self.rate.sleep()
        return True

    def run(self):
        move_cmd = Twist()
        move_cmd.linear.x = LINEAR_SPEED

        turn_cmd = Twist()
        turn_cmd.angular.z = ANGULAR_SPEED
        
        stop_cmd = Twist()

        linear_duration = SIDE_LENGTH / LINEAR_SPEED
        angular_duration = TURN_ANGLE / ANGULAR_SPEED

        for i in range(4):
            rospy.loginfo(f"Side {i+1}: Moving forward.")
            if not self._publish_for_duration(move_cmd, linear_duration): return
            if not self._publish_for_duration(stop_cmd, 0.2): return # Brief stop

            if i < 3: # Turn for the first 3 corners
                rospy.loginfo(f"Side {i+1}: Turning.")
                if not self._publish_for_duration(turn_cmd, angular_duration): return
                if not self._publish_for_duration(stop_cmd, 0.2): return # Brief stop
        
        # Final stop
        for _ in range(HZ // 2): # Stop for 0.5 seconds
             if rospy.is_shutdown(): break
             self.vel_pub.publish(stop_cmd)
             self.rate.sleep()

if __name__ == "__main__":
    try:
        Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Execution interrupted.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
