#!/usr/bin/env python3
"""Step 4: ArUco sign detector — read speed signs from the front camera.

Publishes (BUILDLOG topic contract):
  /race/active_sign     Int32    last CONFIRMED marker id (10/20/30), -1 = none yet
  /race/sign_speed_cap  Float32  speed cap implied by the active sign (m/s)

Detection is per-frame; *confirmation* is temporal: a marker id must be seen
in CONFIRM_FRAMES consecutive frames before it becomes active (rejects
one-frame false positives), and the active sign then stays latched until a
different id is confirmed — passing/losing sight of the sign does NOT clear
it, per the assignment rules.
"""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32

SPEED_CAPS = {10: 1.0, 20: 1.8, 30: 2.5}   # SLOW / NORMAL / BOOST
DEFAULT_CAP = 1.8                           # before any sign: NORMAL rules
CONFIRM_FRAMES = 3                          # consecutive frames to confirm
MIN_AREA_PX = 400.0                         # ignore markers smaller than this


class SignDetector:
    def __init__(self):
        # OpenCV 4.2 ArUco API (container ships 4.2)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        self.candidate_id = None    # id we're currently counting up
        self.candidate_count = 0
        self.active_id = -1

        self.pub_sign = rospy.Publisher("/race/active_sign", Int32,
                                        queue_size=1, latch=True)
        self.pub_cap = rospy.Publisher("/race/sign_speed_cap", Float32,
                                       queue_size=1, latch=True)
        self.pub_sign.publish(Int32(self.active_id))
        self.pub_cap.publish(Float32(DEFAULT_CAP))

        rospy.Subscriber("/camera/front/image_raw", Image, self.on_image,
                         queue_size=1, buff_size=2 ** 22)
        rospy.loginfo("sign_detector: ready (confirm=%d frames)", CONFIRM_FRAMES)

    def on_image(self, msg):
        # Raw Image -> numpy, no cv_bridge needed for simple 8-bit encodings
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                  parameters=self.params)

        seen = None
        if ids is not None:
            # Keep only known race signs, pick the largest (= nearest) one
            best_area = 0.0
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                if int(marker_id) not in SPEED_CAPS:
                    continue
                area = cv2.contourArea(marker_corners.astype(np.float32))
                if area >= MIN_AREA_PX and area > best_area:
                    best_area = area
                    seen = int(marker_id)
            if seen is not None:
                rospy.logdebug("frame: marker %d area %.0f px", seen, best_area)

        self.update_confirmation(seen, msg.header.stamp)

    def update_confirmation(self, seen, stamp):
        if seen is None:
            # No sign this frame: reset the streak, keep the active sign latched
            self.candidate_id = None
            self.candidate_count = 0
            return

        if seen == self.candidate_id:
            self.candidate_count += 1
        else:
            self.candidate_id = seen
            self.candidate_count = 1

        if self.candidate_count == CONFIRM_FRAMES and seen != self.active_id:
            self.active_id = seen
            cap = SPEED_CAPS[seen]
            self.pub_sign.publish(Int32(seen))
            self.pub_cap.publish(Float32(cap))
            rospy.loginfo("[%.2f] SIGN CONFIRMED: %d -> cap %.1f m/s",
                          stamp.to_sec(), seen, cap)


if __name__ == "__main__":
    rospy.init_node("sign_detector")
    SignDetector()
    rospy.spin()
