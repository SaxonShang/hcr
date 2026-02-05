#!/usr/bin/env python3
"""Deprecated helper to synthesize CameraInfo from depth images.

This repository previously carried multiple small scripts for camera_info.
The canonical version is now:
  p3at_nav/scripts/camera_info_from_image_stamp.py

This file is kept only to avoid breaking older notes.
"""

import math
import rospy
from sensor_msgs.msg import CameraInfo, Image


class DepthCameraInfoGenerator:
    def __init__(self):
        # Topics (URDF-consistent defaults)
        self.in_image_topic = rospy.get_param("~in_image_topic", "/sim_p3at/camera/depth/image_rect_raw")
        self.out_info_topic = rospy.get_param("~out_info_topic", "/sim_p3at/camera/depth/camera_info_sync")
        self.frame_id = rospy.get_param("~frame_id", "camera_depth_optical_frame")

        # Intrinsics
        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))
        self.hfov = float(rospy.get_param("~horizontal_fov", 1.211))  # radians

        self.pub = rospy.Publisher(self.out_info_topic, CameraInfo, queue_size=10)
        self.sub = rospy.Subscriber(self.in_image_topic, Image, self._cb, queue_size=10)

        rospy.logwarn(
            "[depth_camera_info_from_image] Deprecated. Use camera_info_from_image_stamp.py. Publishing %s -> %s",
            self.in_image_topic,
            self.out_info_topic,
        )

    def _cb(self, img_msg: Image):
        # Use the image timestamp for perfect sync
        info = CameraInfo()
        info.header.stamp = img_msg.header.stamp
        info.header.frame_id = self.frame_id

        w = self.width if self.width > 0 else img_msg.width
        h = self.height if self.height > 0 else img_msg.height

        fx = w / (2.0 * math.tan(self.hfov / 2.0))
        fy = fx
        cx = w / 2.0
        cy = h / 2.0

        info.width = w
        info.height = h
        info.distortion_model = "plumb_bob"
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.P = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.pub.publish(info)


def main():
    rospy.init_node("depth_camera_info_generator", anonymous=False)
    DepthCameraInfoGenerator()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
