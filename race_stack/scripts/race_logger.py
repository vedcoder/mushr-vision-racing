#!/usr/bin/env python3
"""Step 7: race logger — one CSV row per control tick, per the submission
checklist: time, pose, speed, steering, min LiDAR range, active sign,
state, plus lap/progress/lateral error for the plots.

Writes /assignment/logs/<run_name>_<stamp>.csv (run_name from ~run_name).
Every column is captured from topics — the logger computes nothing except
lateral error (distance to nearest centerline point), so the CSV is an
honest record of what the stack actually published.
"""
import csv
import math
import os

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32, String


class RaceLogger:
    def __init__(self):
        run = rospy.get_param("~run_name", "run")
        csv_path = rospy.get_param("~waypoints_csv",
                                   "/assignment/track/centerline_waypoints.csv")
        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        self.wp = np.array([[float(r["x"]), float(r["y"])] for r in rows])

        os.makedirs("/assignment/logs", exist_ok=True)
        stamp = rospy.Time.now().to_sec()
        self.path = "/assignment/logs/%s_%d.csv" % (run, int(stamp))
        self.f = open(self.path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "x", "y", "yaw", "cmd_speed", "cmd_steer",
                         "min_scan", "active_sign", "state", "lap",
                         "progress_m", "lat_err",
                         "cap_curve", "cap_sign", "cap_lidar"])

        self.d = {"x": 0.0, "y": 0.0, "yaw": 0.0, "cmd_speed": 0.0,
                  "cmd_steer": 0.0, "min_scan": float("nan"),
                  "active_sign": -1, "state": "?", "lap": 0,
                  "progress_m": 0.0, "cap_curve": float("nan"),
                  "cap_sign": float("nan"), "cap_lidar": float("nan")}

        sub = rospy.Subscriber
        sub("/mushr_sim/car/odom", Odometry, self.on_odom, queue_size=1)
        sub("/car/mux/ackermann_cmd_mux/input/navigation",
            AckermannDriveStamped, self.on_cmd, queue_size=1)
        sub("/car/scan", LaserScan, self.on_scan, queue_size=1)
        sub("/race/active_sign", Int32,
            lambda m: self.d.update(active_sign=m.data), queue_size=1)
        sub("/race/state", String,
            lambda m: self.d.update(state=m.data), queue_size=1)
        sub("/race/lap_count", Int32,
            lambda m: self.d.update(lap=m.data), queue_size=1)
        sub("/race/progress_m", Float32,
            lambda m: self.d.update(progress_m=m.data), queue_size=1)
        sub("/race/curve_speed_cap", Float32,
            lambda m: self.d.update(cap_curve=m.data), queue_size=1)
        sub("/race/sign_speed_cap", Float32,
            lambda m: self.d.update(cap_sign=m.data), queue_size=1)
        sub("/race/lidar_speed_cap", Float32,
            lambda m: self.d.update(cap_lidar=m.data), queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.tick)   # 20 Hz rows
        rospy.on_shutdown(self.close)
        rospy.loginfo("race_logger: writing %s", self.path)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.d["x"], self.d["y"] = p.x, p.y
        self.d["yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z))

    def on_cmd(self, msg):
        self.d["cmd_speed"] = msg.drive.speed
        self.d["cmd_steer"] = msg.drive.steering_angle

    def on_scan(self, msg):
        r = np.asarray(msg.ranges)
        valid = np.isfinite(r) & (r > msg.range_min)
        self.d["min_scan"] = float(r[valid].min()) if valid.any() else float("nan")

    def tick(self, _evt):
        lat = float(np.min(np.hypot(self.wp[:, 0] - self.d["x"],
                                    self.wp[:, 1] - self.d["y"])))
        self.w.writerow(["%.3f" % rospy.Time.now().to_sec()] +
                        ["%.4f" % self.d[k] if isinstance(self.d[k], float)
                         else self.d[k]
                         for k in ("x", "y", "yaw", "cmd_speed", "cmd_steer",
                                   "min_scan")] +
                        [self.d["active_sign"], self.d["state"],
                         self.d["lap"], "%.2f" % self.d["progress_m"],
                         "%.4f" % lat,
                         "%.3f" % self.d["cap_curve"],
                         "%.3f" % self.d["cap_sign"],
                         "%.3f" % self.d["cap_lidar"]])

    def close(self):
        self.f.flush()
        self.f.close()
        rospy.loginfo("race_logger: closed %s", self.path)


if __name__ == "__main__":
    rospy.init_node("race_logger")
    RaceLogger()
    rospy.spin()
