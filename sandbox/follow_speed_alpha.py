#!/usr/bin/env python3
"""Sandbox: Pure Pursuit + YOUR speed idea — fast when the carrot is
straight ahead (small alpha), slow when it's off to the side (big alpha).

Identical to follow_waypoints.py except the marked block in run().
"""
import csv
import math

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

WAYPOINTS_CSV = "/assignment/track/centerline_waypoints.csv"
LOOKAHEAD = 1.2
WHEELBASE = 0.33
MAX_STEER = 0.34

# --- speed law parameters (your idea) ---
V_MAX = 2.0     # on a dead-straight
V_MIN = 0.8     # floor so it never stalls mid-corner
K_ALPHA = 4.0   # how aggressively alpha reduces speed


class PurePursuit:
    def __init__(self):
        with open(WAYPOINTS_CSV) as f:
            rows = list(csv.DictReader(f))
        self.wp = np.array([[float(r["x"]), float(r["y"])] for r in rows])
        self.n = len(self.wp)
        self.pose = None
        self.nearest = 0
        self.pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation",
                                   AckermannDriveStamped, queue_size=1)
        rospy.Subscriber("/mushr_sim/car/odom", Odometry, self.on_odom, queue_size=1)
        rospy.loginfo("loaded %d waypoints", self.n)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def pick_carrot(self, x, y):
        d = np.hypot(self.wp[:, 0] - x, self.wp[:, 1] - y)
        self.nearest = int(np.argmin(d))
        for step in range(self.n):
            j = (self.nearest + step) % self.n
            if d[j] >= LOOKAHEAD and step > 0:
                return self.wp[j]
        return self.wp[(self.nearest + 5) % self.n]

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.pose is None:
                rate.sleep()
                continue
            x, y, yaw = self.pose
            cx, cy = self.pick_carrot(x, y)

            dx, dy = cx - x, cy - y
            fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
            left = -dx * math.sin(yaw) + dy * math.cos(yaw)
            alpha = math.atan2(left, fwd)
            dist = math.hypot(fwd, left)

            steer = math.atan(2 * WHEELBASE * math.sin(alpha) / dist)
            steer = max(-MAX_STEER, min(MAX_STEER, steer))

            # ======= YOUR SPEED LAW: alpha small -> fast, alpha big -> slow =======
            speed = V_MAX / (1.0 + K_ALPHA * abs(alpha))
            speed = max(V_MIN, min(V_MAX, speed))
            # ======================================================================

            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.drive.speed = speed
            msg.drive.steering_angle = steer
            self.pub.publish(msg)

            lat_err = float(np.hypot(*(self.wp[self.nearest] - [x, y])))
            rospy.loginfo_throttle(1.0, "wp %d  v %.2f  alpha %+.2f  lat_err %.2f",
                                   self.nearest, speed, alpha, lat_err)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("follow_speed_alpha")
    PurePursuit().run()
