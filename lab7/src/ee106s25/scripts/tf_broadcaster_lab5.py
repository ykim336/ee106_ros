#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
# For converting Euler angles to quaternions if needed,
# though for no rotation, the quaternion is simple.
# from tf.transformations import quaternion_from_euler

def broadcast_static_transforms():
    rospy.init_node('turtlebot_static_tf_broadcaster', anonymous=True)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    transforms_to_broadcast = []

    # 1. Transform from base_footprint to base_scan
    t_footprint_to_scan = geometry_msgs.msg.TransformStamped()
    t_footprint_to_scan.header.stamp = rospy.Time.now()
    t_footprint_to_scan.header.frame_id = "base_footprint"
    t_footprint_to_scan.child_frame_id = "base_scan"
    
    t_footprint_to_scan.transform.translation.x = 0.0
    t_footprint_to_scan.transform.translation.y = 0.0
    t_footprint_to_scan.transform.translation.z = 0.20 # 20cm above
    
    # No relative rotation, so quaternion is (0,0,0,1) (x,y,z,w)
    t_footprint_to_scan.transform.rotation.x = 0.0
    t_footprint_to_scan.transform.rotation.y = 0.0
    t_footprint_to_scan.transform.rotation.z = 0.0
    t_footprint_to_scan.transform.rotation.w = 1.0
    transforms_to_broadcast.append(t_footprint_to_scan)

    # 2. Transform from base_scan to left_limit
    t_scan_to_left = geometry_msgs.msg.TransformStamped()
    t_scan_to_left.header.stamp = rospy.Time.now()
    t_scan_to_left.header.frame_id = "base_scan"
    t_scan_to_left.child_frame_id = "left_limit"
    
    t_scan_to_left.transform.translation.x = 0.0
    t_scan_to_left.transform.translation.y = 0.07 # 7cm to the left (positive Y)
    t_scan_to_left.transform.translation.z = 0.0  # Same Z level as base_scan
    
    t_scan_to_left.transform.rotation.x = 0.0
    t_scan_to_left.transform.rotation.y = 0.0
    t_scan_to_left.transform.rotation.z = 0.0
    t_scan_to_left.transform.rotation.w = 1.0
    transforms_to_broadcast.append(t_scan_to_left)

    # 3. Transform from base_scan to right_limit
    t_scan_to_right = geometry_msgs.msg.TransformStamped()
    t_scan_to_right.header.stamp = rospy.Time.now()
    t_scan_to_right.header.frame_id = "base_scan"
    t_scan_to_right.child_frame_id = "right_limit"
    
    t_scan_to_right.transform.translation.x = 0.0
    t_scan_to_right.transform.translation.y = -0.07 # 7cm to the right (negative Y)
    t_scan_to_right.transform.translation.z = 0.0   # Same Z level as base_scan
    
    t_scan_to_right.transform.rotation.x = 0.0
    t_scan_to_right.transform.rotation.y = 0.0
    t_scan_to_right.transform.rotation.z = 0.0
    t_scan_to_right.transform.rotation.w = 1.0
    transforms_to_broadcast.append(t_scan_to_right)
    
    static_broadcaster.sendTransform(transforms_to_broadcast)
    rospy.loginfo("Static transforms (base_scan, left_limit, right_limit) have been broadcasted.")
    rospy.spin() # Keep the node alive

if __name__ == '__main__':
    try:
        broadcast_static_transforms()
    except rospy.ROSInterruptException:
        rospy.loginfo("Static transform broadcaster shutting down.")