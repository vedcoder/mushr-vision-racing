#!/usr/bin/env python3
"""Step 3: curvature-aware speed planner.

Publishes /race/curve_speed_cap (Float32): the speed the track geometry
allows, looking BRAKE_LOOKAHEAD_M ahead so braking happens before corners,
not in them (the measured flaw of the reactive alpha law).

Curvature at waypoint i uses the CSV's own headings:
    kappa_i = wrap(yaw[i+1] - yaw[i-1]) / (s[i+1] - s[i-1])
i.e. how much the path direction changes per metre of travel. Straight:
kappa ~ 0. Hairpin: |kappa| ~ 1-2 (radius 0.5-1 m). The speed law is the
handout's: v = v_max / (1 + k*|kappa|), clipped to [v_min, v_max].
Everything is precomputed; at runtime this node is one array lookup fed by
/race/nearest_waypoint.
"""
import csv
import math

import numpy as np
import rospy
from std_msgs.msg import Float32, Int32


class SpeedPlanner:
    def __init__(self):
        v_max = rospy.get_param("speed_planner/v_max", 2.5)
        v_min = rospy.get_param("speed_planner/v_min", 0.7)
        k = rospy.get_param("speed_planner/k_kappa", 1.6)
        look_m = rospy.get_param("speed_planner/brake_lookahead_m", 3.0)
        csv_path = rospy.get_param("~waypoints_csv",
                                   "/assignment/track/centerline_waypoints.csv")

        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
        yaw = np.array([float(r["yaw"]) for r in rows])
        n = len(xy)

        # arc length between i-1 and i+1 (wrapping the closed loop)
        seg = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)  # seg[i]: i -> i+1
        ds = seg + np.roll(seg, 1)                          # (i-1->i) + (i->i+1)
        dyaw = np.roll(yaw, -1) - np.roll(yaw, 1)
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi   # wrap to [-pi, pi)
        kappa = np.abs(dyaw / ds)

        v_curve = np.clip(v_max / (1.0 + k * kappa), v_min, v_max)

        # Brake-ahead: cap at waypoint i = the MINIMUM allowed speed over the
        # next `look_m` metres of track, so we arrive at corners already slow.
        window = max(1, int(round(look_m / float(np.mean(seg)))))
        self.cap = np.array([v_curve.take(range(i, i + window + 1),
                                          mode="wrap").min()
                             for i in range(n)])

        self.pub = rospy.Publisher("/race/curve_speed_cap", Float32,
                                   queue_size=1)
        rospy.Subscriber("/race/nearest_waypoint", Int32, self.on_nearest,
                         queue_size=1)
        rospy.loginfo("speed_planner: kappa max %.2f, cap range %.2f-%.2f, "
                      "window %d wp", kappa.max(), self.cap.min(),
                      self.cap.max(), window)

    def on_nearest(self, msg):
        self.pub.publish(Float32(float(self.cap[msg.data % len(self.cap)])))


if __name__ == "__main__":
    rospy.init_node("speed_planner")
    SpeedPlanner()
    rospy.spin()
