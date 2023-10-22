#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

rospy.init_node('empty_odometry_publisher')
empty_odometry = Odometry()
empty_odometry.pose.pose.orientation.w = 1.0;
odom_pub = rospy.Publisher('/atom/odometry', Odometry, queue_size=10)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    empty_odometry.header.stamp = rospy.Time.now()
    odom_pub.publish(empty_odometry)
    rate.sleep()