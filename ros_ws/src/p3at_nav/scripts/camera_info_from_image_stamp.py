#!/usr/bin/env python3
"""Publish a CameraInfo message whose timestamp matches a depth Image message.

Some Gazebo camera plugins publish camera_info at 0 Hz or with timestamps that do not match
the corresponding depth images. depthimage_to_laserscan relies on time-synchronized
Image and CameraInfo.

This node:
  - Subscribes to a depth image topic.
  - Publishes a synthetic CameraInfo message for every image, copying header.stamp.

The intrinsic matrix is computed from image width and horizontal FOV.
"""

import math
import rospy
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoFromImageStamp:
    def __init__(self):
        # Camera intrinsics (match p3at_with_depth_camera.urdf.xacro)
        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))
        self.hfov = float(rospy.get_param("~horizontal_fov", 1.211))  # radians
        self.frame_id = rospy.get_param("~frame_id", "camera_depth_optical_frame")

        # Topics (match the URDF plugin names)
        self.in_image_topic = rospy.get_param(
            "~in_image_topic", "/sim_p3at/camera/depth/image_rect_raw"
        )
        # NOTE: depthimage_to_laserscan (CameraSubscriber) expects CameraInfo on
        # /sim_p3at/camera/depth/camera_info when the depth image topic is
        # /sim_p3at/camera/depth/image_rect_raw.
        self.out_info_topic = rospy.get_param(
            "~out_info_topic", "/sim_p3at/camera/depth/camera_info"
        )

        # Compute pinhole intrinsics from HFOV and image width (assume square pixels)
        fx = self.width / (2.0 * math.tan(self.hfov / 2.0))
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0

        # Prepare a CameraInfo template (only header changes per frame)
        self.info = CameraInfo()
        self.info.width = self.width
        self.info.height = self.height
        self.info.distortion_model = "plumb_bob"
        self.info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.info.K = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        self.info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        self.info.P = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        self.pub = rospy.Publisher(self.out_info_topic, CameraInfo, queue_size=10)
        self.sub = rospy.Subscriber(self.in_image_topic, Image, self._cb, queue_size=10)

        rospy.loginfo(
            "[camera_info_from_image_stamp] %s -> %s (frame_id=%s)",
            self.in_image_topic,
            self.out_info_topic,
            self.frame_id,
        )

    def _cb(self, img: Image):
        self.info.header.stamp = img.header.stamp
        self.info.header.frame_id = self.frame_id
        self.pub.publish(self.info)


def main():
    rospy.init_node("camera_info_from_image_stamp", anonymous=True)
    CameraInfoFromImageStamp()
    rospy.spin()


if __name__ == "__main__":
    main()
