#!/usr/bin/env python3
"""Step 1: waypoint manager — where are we on the track, and how many laps?

Publishes (see BUILDLOG topic contract):
  /race/nearest_waypoint  Int32    index of the closest centerline point
  /race/progress_m        Float32  metres travelled along the centerline
  /race/lap_count         Int32    completed laps (latched)

Lap detection uses hysteresis: a lap counts only when the nearest index
jumps from the last LAP_ZONE fraction of the track into the first — i.e.
the car genuinely crossed the start line going forward. Crossing backwards
decrements, so shuttling over the line can never inflate the count.
"""
import csv
import math

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker

LAP_ZONE = 0.1  # fraction of track length treated as "just before/after the line"


class WaypointManager:
    def __init__(self):
        csv_path = rospy.get_param("~waypoints_csv",
                                   "/assignment/track/centerline_waypoints.csv")
        odom_topic = rospy.get_param("~odom_topic", "/mushr_sim/car/odom")

        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        self.wp = np.array([[float(r["x"]), float(r["y"])] for r in rows])
        self.n = len(self.wp)

        # Cumulative distance along the centerline: index -> metres of progress.
        seg = np.hypot(np.diff(self.wp[:, 0]), np.diff(self.wp[:, 1]))
        self.cum_dist = np.concatenate([[0.0], np.cumsum(seg)])
        closing = math.hypot(*(self.wp[0] - self.wp[-1]))
        self.track_len = float(self.cum_dist[-1] + closing)

        self.last_idx = None
        self.laps = 0

        self.pub_idx = rospy.Publisher("/race/nearest_waypoint", Int32, queue_size=1)
        self.pub_prog = rospy.Publisher("/race/progress_m", Float32, queue_size=1)
        self.pub_lap = rospy.Publisher("/race/lap_count", Int32,
                                       queue_size=1, latch=True)
        self.pub_lap.publish(Int32(0))

        # Centerline as a latched Marker so Foxglove/RViz can draw it on the map
        self.pub_viz = rospy.Publisher("/race/waypoints_viz", Marker,
                                       queue_size=1, latch=True)
        self.publish_centerline_marker()

        rospy.Subscriber(odom_topic, Odometry, self.on_odom, queue_size=1)
        rospy.loginfo("waypoint_manager: %d waypoints, track length %.1f m",
                      self.n, self.track_len)

    def publish_centerline_marker(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "centerline"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05                       # line width (m)
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.2, 1.0
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in self.wp]
        m.points.append(m.points[0])           # close the loop
        self.pub_viz.publish(m)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        d2 = (self.wp[:, 0] - p.x) ** 2 + (self.wp[:, 1] - p.y) ** 2
        idx = int(np.argmin(d2))

        if self.last_idx is not None:
            hi = (1.0 - LAP_ZONE) * self.n
            lo = LAP_ZONE * self.n
            if self.last_idx > hi and idx < lo:      # forward across the line
                self.laps += 1
                self.pub_lap.publish(Int32(self.laps))
                rospy.loginfo("lap %d complete", self.laps)
            elif idx > hi and self.last_idx < lo:    # backwards across the line
                self.laps -= 1
                self.pub_lap.publish(Int32(self.laps))
                rospy.logwarn("crossed start line backwards (laps=%d)", self.laps)
        self.last_idx = idx

        self.pub_idx.publish(Int32(idx))
        self.pub_prog.publish(Float32(float(self.cum_dist[idx])))


if __name__ == "__main__":
    rospy.init_node("waypoint_manager")
    WaypointManager()
    rospy.spin()
