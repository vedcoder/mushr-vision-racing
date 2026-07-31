#!/usr/bin/env python3
"""Step 5: LiDAR safety — clearance speed cap, emergency stop, dodge bias.

Publishes (BUILDLOG topic contract):
  /race/lidar_speed_cap  Float32  max safe speed given forward clearance (m/s)
  /race/dodge_steer      Float32  steering bias toward free space (rad, +left)

Three escalating behaviours, all derived from one 360-degree scan:
  1. Speed cap: full speed when clear, linear ramp down as forward
     clearance shrinks, zero at STOP_DIST (emergency stop).
  2. E-stop is just the bottom of that ramp — no separate mechanism.
  3. Dodge: when the centre of the forward view is blocked but the sides
     are not, bias steering toward the side with more room. The bias adds
     to Pure Pursuit's steering, so the carrot keeps pulling the car back
     to the racing line once the obstacle is passed.

Noise handling (learned in sandbox/move_until_wall.py): the sim's laser
model produces phantom single-ray short readings, so every clearance here
is a low PERCENTILE of a ray window, never a raw minimum.
"""
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

STOP_DIST = 0.45     # clearance (m) at which commanded speed reaches 0
FREE_DIST = 2.50     # clearance (m) at which no speed limit applies
MAX_CAP = 99.0       # published cap when unconstrained ("no opinion")

# +/-0.12 rad (~7 deg): wide enough to cover the car's width at braking
# range, narrow enough not to graze the walls of this 2.3 m-wide track
# (at 11 deg the cap was binding at ~1.9 on open straights — wall capture).
CENTER_HALF = 0.12
SIDE_IN = 0.20       # side windows: 0.20..0.70 rad (~11..40 deg) each side
SIDE_OUT = 0.70

DODGE_TRIGGER = 1.6  # start dodging when centre clearance drops below this
DODGE_MAX = 0.20     # max steering bias (rad) — never exceeds servo range
PERCENTILE = 10      # robust clearance = this percentile of the window


class LidarSafety:
    def __init__(self):
        self.pub_cap = rospy.Publisher("/race/lidar_speed_cap", Float32,
                                       queue_size=1)
        self.pub_dodge = rospy.Publisher("/race/dodge_steer", Float32,
                                         queue_size=1)
        rospy.Subscriber("/car/scan", LaserScan, self.on_scan, queue_size=1)
        self.angles = None
        rospy.loginfo("lidar_safety: stop<%.2fm free>%.2fm dodge<%.1fm",
                      STOP_DIST, FREE_DIST, DODGE_TRIGGER)

    def window_clearance(self, ranges, valid, lo, hi):
        """Robust clearance (m) inside the angular window [lo, hi) rad."""
        sel = valid & (self.angles >= lo) & (self.angles < hi)
        window = ranges[sel]
        if window.size == 0:
            return FREE_DIST          # no data: assume open, cap won't bind
        return float(np.percentile(window, PERCENTILE))

    def on_scan(self, scan):
        ranges = np.asarray(scan.ranges)
        if self.angles is None or self.angles.size != ranges.size:
            # Ray angles are fixed per scanner config: compute once, reuse
            self.angles = scan.angle_min + \
                np.arange(ranges.size) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges > scan.range_min)

        center = self.window_clearance(ranges, valid, -CENTER_HALF, CENTER_HALF)
        left = self.window_clearance(ranges, valid, SIDE_IN, SIDE_OUT)
        right = self.window_clearance(ranges, valid, -SIDE_OUT, -SIDE_IN)

        # --- speed cap: linear ramp between STOP_DIST and FREE_DIST ---
        if center >= FREE_DIST:
            cap = MAX_CAP
        else:
            frac = (center - STOP_DIST) / (FREE_DIST - STOP_DIST)
            cap = max(0.0, frac) * 2.5      # 2.5 = highest legal race speed
        self.pub_cap.publish(Float32(cap))

        # --- dodge: push toward the freer side, harder the closer we are ---
        if center < DODGE_TRIGGER:
            urgency = 1.0 - (center - STOP_DIST) / (DODGE_TRIGGER - STOP_DIST)
            urgency = min(1.0, max(0.0, urgency))
            direction = 1.0 if left > right else -1.0   # +left, -right
            dodge = direction * urgency * DODGE_MAX
        else:
            dodge = 0.0
        self.pub_dodge.publish(Float32(dodge))

        if cap < 2.5:
            rospy.loginfo_throttle(
                1.0, "clear c%.2f L%.2f R%.2f -> cap %.2f dodge %+.2f",
                center, left, right, cap, dodge)


if __name__ == "__main__":
    rospy.init_node("lidar_safety")
    LidarSafety()
    rospy.spin()
