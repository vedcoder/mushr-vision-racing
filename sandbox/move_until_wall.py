#!/usr/bin/env python3
"""Sandbox: drive forward until the LiDAR sees something close ahead, stop.

This is the seed of the real safety node: read the scan, measure forward
clearance, and let that clearance veto the speed command.
"""
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

STOP_DISTANCE = 0.6   # halt when forward clearance drops below this (m)
FORWARD_CONE = 0.35   # rays within +/- this angle (rad) count as "ahead"
SPEED = 0.5


class MoveUntilWall:
    def __init__(self):
        self.clearance = None
        self.close_rays = 0
        self.pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation",
                                   AckermannDriveStamped, queue_size=1)
        rospy.Subscriber("/car/scan", LaserScan, self.on_scan, queue_size=1)

    def on_scan(self, scan):
        # Each ray's angle: angle_min + index * increment; angle 0 = straight ahead
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.asarray(scan.ranges)
        ahead = np.abs(angles) < FORWARD_CONE
        valid = np.isfinite(ranges) & (ranges > scan.range_min)  # drop NaN misses
        window = ranges[ahead & valid]
        if window.size == 0:
            self.clearance = float("inf")
            return
        # The sim's noise model produces occasional phantom short readings on
        # single rays. A real obstacle spans many adjacent rays, so a lone
        # short ray must not trigger a stop: count rays below the threshold
        # and use a low percentile (not the raw min) as the clearance value.
        self.close_rays = int((window <= STOP_DISTANCE).sum())
        self.clearance = float(np.percentile(window, 10))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            if self.clearance is None:
                msg.drive.speed = 0.0        # no scan received yet: don't move
            elif self.close_rays >= 5:  # >=5 agreeing rays = real obstacle
                msg.drive.speed = 0.0
                self.pub.publish(msg)
                rospy.loginfo("obstacle at %.2f m (%d rays) -> STOPPED",
                              self.clearance, self.close_rays)
                return
            else:
                msg.drive.speed = SPEED
                rospy.loginfo_throttle(1.0, "clearance %.2f m", self.clearance)
            self.pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("move_until_wall")
    MoveUntilWall().run()
