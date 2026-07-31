#!/usr/bin/env python3
"""Safety test: drive dead straight obeying ONLY the LiDAR cap.

No Pure Pursuit, no dodge — steering locked to zero, so the only thing
between the car and the obstacle is /race/lidar_speed_cap. Prints a
(distance-travelled, cap, commanded-speed) table and exits once the cap
stops the car. This is the e-stop + braking-ramp evidence for the report.
"""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

CRUISE = 0.6

rospy.init_node("safety_test")
state = {"cap": 99.0, "x": None, "x0": None}
rospy.Subscriber("/race/lidar_speed_cap", Float32,
                 lambda m: state.update(cap=m.data), queue_size=1)
rospy.Subscriber("/mushr_sim/car/odom", Odometry,
                 lambda m: state.update(x=m.pose.pose.position.x), queue_size=1)
pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/navigation",
                      AckermannDriveStamped, queue_size=1)

rate = rospy.Rate(10)
stopped_ticks = 0
while not rospy.is_shutdown():
    if state["x"] is not None and state["x0"] is None:
        state["x0"] = state["x"]
    speed = min(CRUISE, max(0.0, state["cap"]))
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.drive.speed = speed
    msg.drive.steering_angle = 0.0
    pub.publish(msg)
    if state["x0"] is not None:
        print("travelled %.2f m  cap %.2f  cmd %.2f"
              % (state["x"] - state["x0"], state["cap"], speed), flush=True)
    stopped_ticks = stopped_ticks + 1 if speed < 0.03 else 0
    if stopped_ticks >= 10:
        print("E-STOP HELD: car halted by lidar cap alone", flush=True)
        break
    rate.sleep()
