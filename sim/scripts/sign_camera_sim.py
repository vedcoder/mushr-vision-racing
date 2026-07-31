#!/usr/bin/env python3
"""Synthetic forward-camera simulator for the vision racing assignment.

Renders the ArUco sign boards from signs/ into a virtual pinhole camera
image based on the car's pose, and publishes it on /camera/front/image_raw.
This stands in for the RGB camera that the stock MuSHR simulator lacks.

Signs are placed from config/vision_sign_layouts.yaml (layout selected via
the ~layout param, default "development"). Each board is treated as a
0.52 m x 0.62 m planar billboard at its (x, y, yaw) pose, facing along its
yaw direction; it is rendered with a homography warp so scale and skew
behave like a real camera, which keeps ArUco detection realistic.
"""
import math
from pathlib import Path

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

# Board geometry (must match signs/*.png: 520x620 px board, 0.001 m/px)
BOARD_W, BOARD_H = 0.52, 0.62
BOARD_BASE_HEIGHT = 0.10  # bottom edge height above ground

# Camera intrinsics/pose on the car
IMG_W, IMG_H = 640, 480
# 70 deg keeps enough px/deg for oblique boards; 90 was tried and lost the
# BOOST sign (fewer px per degree) without gaining SLOW (~90 deg off-axis).
FOV_X = math.radians(70.0)
FX = (IMG_W / 2) / math.tan(FOV_X / 2)
FY = FX
CX, CY = IMG_W / 2, IMG_H / 2
CAM_HEIGHT = 0.20      # camera height above ground
CAM_FORWARD = 0.15     # camera ahead of base_link origin
MAX_VIEW_DIST = 8.0    # don't render boards farther than this


class SignCameraSim:
    def __init__(self):
        root = Path(rospy.get_param("~assignment_root",
                                    str(Path(__file__).resolve().parents[2])))
        layout_name = rospy.get_param("~layout", "development")
        layouts = yaml.safe_load(
            (root / "config" / "vision_sign_layouts.yaml").read_text())
        self.signs = layouts[layout_name]
        self.textures = {
            s["marker_id"]: cv2.imread(str(next(
                (root / "signs").glob(f"aruco_{s['marker_id']}_*.png"))))
            for s in self.signs
        }
        self.pose = None
        self.bridge_encoding = "bgr8"
        self.pub = rospy.Publisher("/camera/front/image_raw", Image,
                                   queue_size=1)
        odom_topic = rospy.get_param("~odom_topic", "/mushr_sim/car/odom")
        rospy.Subscriber(odom_topic, Odometry, self.on_odom, queue_size=1)
        hz = rospy.get_param("~rate", 15.0)
        rospy.Timer(rospy.Duration(1.0 / hz), self.render)
        rospy.loginfo("sign_camera_sim: layout=%s signs=%d odom=%s",
                      layout_name, len(self.signs), odom_topic)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def world_to_cam(self, pt, cam_xy, cam_yaw):
        """World point -> camera frame (x right, y down, z forward)."""
        dx, dy, dz = pt[0] - cam_xy[0], pt[1] - cam_xy[1], pt[2] - CAM_HEIGHT
        fwd = dx * math.cos(cam_yaw) + dy * math.sin(cam_yaw)
        left = -dx * math.sin(cam_yaw) + dy * math.cos(cam_yaw)
        return np.array([-left, -dz, fwd])

    def render(self, _evt):
        if self.pose is None:
            return
        px, py, pyaw = self.pose
        cam_xy = (px + CAM_FORWARD * math.cos(pyaw),
                  py + CAM_FORWARD * math.sin(pyaw))
        img = np.full((IMG_H, IMG_W, 3), (120, 130, 135), np.uint8)
        img[int(IMG_H * 0.55):, :] = (95, 100, 105)  # ground plane

        # Render farthest-first so nearer boards overdraw
        order = sorted(self.signs, key=lambda s: -math.hypot(s["x"] - px,
                                                             s["y"] - py))
        for s in order:
            dist = math.hypot(s["x"] - px, s["y"] - py)
            if dist > MAX_VIEW_DIST or dist < 0.05:
                continue
            # Board corners in world (board plane normal to its yaw)
            n = (math.cos(s["yaw"]), math.sin(s["yaw"]))
            # Facing check: board only visible from its front side
            if (px - s["x"]) * n[0] + (py - s["y"]) * n[1] < 0:
                continue
            t = (-n[1], n[0])  # board's lateral direction
            half = BOARD_W / 2
            z0, z1 = BOARD_BASE_HEIGHT, BOARD_BASE_HEIGHT + BOARD_H
            corners_w = [
                (s["x"] - t[0] * half, s["y"] - t[1] * half, z1),  # top-left
                (s["x"] + t[0] * half, s["y"] + t[1] * half, z1),  # top-right
                (s["x"] + t[0] * half, s["y"] + t[1] * half, z0),  # bot-right
                (s["x"] - t[0] * half, s["y"] - t[1] * half, z0),  # bot-left
            ]
            pts = []
            behind = False
            for cw in corners_w:
                c = self.world_to_cam(cw, cam_xy, pyaw)
                if c[2] < 0.1:
                    behind = True
                    break
                pts.append([CX + FX * c[0] / c[2], CY + FY * c[1] / c[2]])
            if behind:
                continue
            dst = np.array(pts, np.float32)
            if cv2.contourArea(dst.astype(np.int32)) < 40:
                continue  # too small / degenerate
            tex = self.textures[s["marker_id"]]
            h, w = tex.shape[:2]
            src = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]],
                           np.float32)
            H, _ = cv2.findHomography(src, dst)
            if H is None:
                continue
            warped = cv2.warpPerspective(tex, H, (IMG_W, IMG_H))
            mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), H,
                                       (IMG_W, IMG_H))
            img[mask > 0] = warped[mask > 0]

        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_front"
        msg.height, msg.width = IMG_H, IMG_W
        msg.encoding = self.bridge_encoding
        msg.step = IMG_W * 3
        msg.data = img.tobytes()
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("sign_camera_sim")
    SignCameraSim()
    rospy.spin()
