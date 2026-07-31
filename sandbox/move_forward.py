#!/usr/bin/env python3
"""Sandbox experiment: drive the car forward at constant speed.

The mux stops the car ~0.2 s after we stop publishing, so commands must be
streamed continuously (10 Hz here), not sent once.
"""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

rospy.init_node("move_forward")
pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation",
                      AckermannDriveStamped, queue_size=1)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.drive.speed = 0.5             # m/s forward
    msg.drive.steering_angle = 0.0    # radians, 0 = straight
    pub.publish(msg)
    rate.sleep()
