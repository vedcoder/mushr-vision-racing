#!/usr/bin/env python3
"""Step 6: command arbiter + recovery FSM — the only node that drives.

Speed:    v = min(v_curve, v_sign, v_lidar), then rate-limited so commands
          are physically smooth (the handout's required accel limits).
Steering: blend of Pure Pursuit and the dodge, weighted by dodge urgency:
              u = |dodge| / DODGE_MAX          (0 = clear, 1 = about to stop)
              steer = (1-u) * pp + sign(dodge) * u * max_steer
          At full urgency the dodge gets FULL steering authority — the fix
          for the evaluation-B deadlock, where a bounded additive bias
          could never out-vote Pure Pursuit aiming through an obstacle.
Recovery: RACING -> STUCK (commanded > 0 but not moving for stuck_seconds)
          -> REVERSING (back away, steered to swing the nose toward free
          space) -> RACING. Publishes /race/state for logging/graders.
"""
import math

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

DODGE_MAX = 0.20   # must match lidar_safety.py


class Arbiter:
    def __init__(self):
        self.max_steer = rospy.get_param("path_follower/max_steer", 0.34)
        self.accel = rospy.get_param("arbiter/accel_limit", 1.5)
        self.decel = rospy.get_param("arbiter/decel_limit", 3.0)
        self.stuck_s = rospy.get_param("arbiter/stuck_seconds", 2.0)
        self.rev_s = rospy.get_param("arbiter/reverse_seconds", 1.8)
        self.rev_v = rospy.get_param("arbiter/reverse_speed", 0.6)

        self.pp = 0.0
        self.dodge = 0.0
        self.caps = {"curve": 2.5, "sign": 1.8, "lidar": 99.0}
        self.pose = None
        self.last_pose = None
        self.last_move_t = rospy.get_time()
        self.state = "RACING"
        self.state_t = rospy.get_time()
        self.rev_steer = 0.0
        self.cmd_v = 0.0

        sub = rospy.Subscriber
        sub("/race/pp_steer", Float32,
            lambda m: setattr(self, "pp", m.data), queue_size=1)
        sub("/race/dodge_steer", Float32,
            lambda m: setattr(self, "dodge", m.data), queue_size=1)
        sub("/race/curve_speed_cap", Float32,
            lambda m: self.caps.update(curve=m.data), queue_size=1)
        sub("/race/sign_speed_cap", Float32,
            lambda m: self.caps.update(sign=m.data), queue_size=1)
        sub("/race/lidar_speed_cap", Float32,
            lambda m: self.caps.update(lidar=m.data), queue_size=1)
        sub(rospy.get_param("~odom_topic", "/mushr_sim/car/odom"),
            Odometry, self.on_odom, queue_size=1)

        self.pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation",
                                   AckermannDriveStamped, queue_size=1)
        self.pub_v = rospy.Publisher("/race/target_speed", Float32, queue_size=1)
        self.pub_state = rospy.Publisher("/race/state", String,
                                         queue_size=1, latch=True)
        self.pub_state.publish(String(self.state))
        rospy.Timer(rospy.Duration(0.05), self.step)   # 20 Hz
        rospy.loginfo("arbiter: up (accel %.1f decel %.1f)", self.accel, self.decel)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.pose is not None:
            if math.hypot(p.x - self.pose[0], p.y - self.pose[1]) > 0.01:
                self.last_move_t = rospy.get_time()   # we are moving
        self.pose = (p.x, p.y)

    def set_state(self, new):
        if new != self.state:
            rospy.loginfo("state %s -> %s", self.state, new)
            self.state = new
            self.state_t = rospy.get_time()
            self.pub_state.publish(String(new))

    def step(self, _evt):
        now = rospy.get_time()
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()

        if self.state == "RACING":
            target = min(self.caps["curve"], self.caps["sign"],
                         self.caps["lidar"])
            # rate limiting: physically smooth commands
            dt = 0.05
            target = min(target, self.cmd_v + self.accel * dt)
            target = max(target, self.cmd_v - self.decel * dt)
            self.cmd_v = max(0.0, target)

            u = min(1.0, abs(self.dodge) / DODGE_MAX)
            steer = (1.0 - u) * self.pp + \
                math.copysign(u * self.max_steer, self.dodge) if u > 0 \
                else self.pp
            msg.drive.speed = self.cmd_v
            msg.drive.steering_angle = max(-self.max_steer,
                                           min(self.max_steer, steer))

            # stuck: not moving for too long while either commanding motion
            # or pinned by the lidar e-stop (cmd_v=0 there, so it alone can
            # never satisfy a cmd_v>0.1 condition — learned in testing)
            pinned = self.caps["lidar"] < 0.3
            if (self.cmd_v > 0.1 or pinned) and \
                    now - self.last_move_t > self.stuck_s:
                # back up steering opposite to where the nose should swing:
                # if dodge points left (+), reversing with right steer (-)
                # swings the nose left. No dodge signal -> use pp direction.
                ref = self.dodge if abs(self.dodge) > 0.02 else self.pp
                self.rev_steer = -math.copysign(0.3, ref if ref != 0 else 1.0)
                self.set_state("REVERSING")

        elif self.state == "REVERSING":
            msg.drive.speed = -self.rev_v
            msg.drive.steering_angle = self.rev_steer
            self.cmd_v = 0.0
            if now - self.state_t > self.rev_s:
                self.last_move_t = now       # fresh start for stuck timer
                self.set_state("RACING")

        self.pub.publish(msg)
        self.pub_v.publish(Float32(msg.drive.speed))


if __name__ == "__main__":
    rospy.init_node("arbiter")
    Arbiter()
    rospy.spin()
