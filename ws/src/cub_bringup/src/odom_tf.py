#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry

def odom_callback(odom_msg):
    # Create a TF broadcaster
    tf_broadcaster = tf.TransformBroadcaster()

    # Get the pose from the received Odometry message
    pose = odom_msg.pose.pose

    # Get the timestamp from the Odometry message
    timestamp = odom_msg.header.stamp

    # Publish the TF transform from odom to base_link
    tf_broadcaster.sendTransform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        timestamp,
        "base_footprint",
        "odometry"
    )

if __name__ == '__main__':
    rospy.init_node('odom_to_base_link_tf_node')
    rospy.Subscriber('/atom/odometry', Odometry, odom_callback)
    rospy.spin()