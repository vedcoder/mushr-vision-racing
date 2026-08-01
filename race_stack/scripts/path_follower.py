#!/usr/bin/env python3
"""Step 2: Pure Pursuit path follower — steering opinion only.

Publishes /race/pp_steer (Float32, rad). Does NOT command the car and does
NOT decide speed: the arbiter owns the actual drive command. This is the
sandbox follower's steering half, formalized: parameters from the ROS
param server (race_stack/config/params.yaml), waypoints from the same CSV
as everyone else.
"""
import csv
import math

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class PathFollower:
    def __init__(self):
        self.lookahead = rospy.get_param("path_follower/lookahead", 1.2)
        self.wheelbase = rospy.get_param("path_follower/wheelbase", 0.33)
        self.max_steer = rospy.get_param("path_follower/max_steer", 0.34)
        csv_path = rospy.get_param("~waypoints_csv",
                                   "/assignment/track/centerline_waypoints.csv")
        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        self.wp = np.array([[float(r["x"]), float(r["y"])] for r in rows])
        self.n = len(self.wp)

        self.pub = rospy.Publisher("/race/pp_steer", Float32, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/mushr_sim/car/odom"),
                         Odometry, self.on_odom, queue_size=1)
        rospy.loginfo("path_follower: lookahead=%.2f", self.lookahead)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))

        d = np.hypot(self.wp[:, 0] - p.x, self.wp[:, 1] - p.y)
        nearest = int(np.argmin(d))
        carrot = self.wp[(nearest + 5) % self.n]
        for step in range(1, self.n):
            j = (nearest + step) % self.n
            if d[j] >= self.lookahead:
                carrot = self.wp[j]
                break

        dx, dy = carrot[0] - p.x, carrot[1] - p.y
        fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        alpha = math.atan2(left, fwd)
        dist = math.hypot(fwd, left)

        steer = math.atan(2 * self.wheelbase * math.sin(alpha) / dist)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        self.pub.publish(Float32(steer))


if __name__ == "__main__":
    rospy.init_node("path_follower")
    PathFollower()
    rospy.spin()
