#!/usr/bin/env python3
"""Sandbox: Pure Pursuit — follow the centerline waypoints at constant speed.

The idea: aim at a "carrot" point on the centerline ~LOOKAHEAD metres ahead
of the car. Circle geometry gives the steering angle that arcs the car
through that point. As the car advances the carrot slides along the track,
pulling the car around the lap.
"""
import csv
import math

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

WAYPOINTS_CSV = "/assignment/track/centerline_waypoints.csv"
LOOKAHEAD = 1.2      # carrot distance (m): shorter = tighter tracking, twitchier
WHEELBASE = 0.33     # MuSHR axle-to-axle length (m), from the URDF
SPEED = 1.0          # constant for now — the speed planner comes later
MAX_STEER = 0.34     # servo limit (rad)


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
        """Nearest waypoint, then walk forward to the first one >= LOOKAHEAD away."""
        d = np.hypot(self.wp[:, 0] - x, self.wp[:, 1] - y)
        self.nearest = int(np.argmin(d))
        i = self.nearest
        for step in range(self.n):
            j = (self.nearest + step) % self.n   # wraps around at the finish line
            if d[j] >= LOOKAHEAD and step > 0:
                return self.wp[j]
        return self.wp[(self.nearest + 5) % self.n]  # fallback: a bit ahead

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.pose is None:
                rate.sleep()
                continue
            x, y, yaw = self.pose
            cx, cy = self.pick_carrot(x, y)

            # Carrot position in the car's own frame (x forward, y left)
            dx, dy = cx - x, cy - y
            fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
            left = -dx * math.sin(yaw) + dy * math.cos(yaw)
            alpha = math.atan2(left, fwd)              # bearing to the carrot
            dist = math.hypot(fwd, left)

            # The Pure Pursuit formula: arc through the carrot
            steer = math.atan(2 * WHEELBASE * math.sin(alpha) / dist)
            steer = max(-MAX_STEER, min(MAX_STEER, steer))

            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.drive.speed = SPEED
            msg.drive.steering_angle = steer
            self.pub.publish(msg)

            # lateral error = distance to nearest centerline point (for tuning)
            lat_err = float(np.hypot(*(self.wp[self.nearest] - [x, y])))
            rospy.loginfo_throttle(2.0, "wp %d/%d  lat_err %.2f m  steer %+.2f",
                                   self.nearest, self.n, lat_err, steer)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("follow_waypoints")
    PurePursuit().run()
