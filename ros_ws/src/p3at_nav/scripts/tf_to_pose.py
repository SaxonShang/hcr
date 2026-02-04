#!/usr/bin/env python3
"""Publish PoseStamped from TF.

This node bridges TF into a PoseStamped topic.

Why this exists in the integrated stack:
- map_manager (from the HCR codebase) can consume geometry_msgs/PoseStamped in the map frame.
- When running SLAM (GMapping), /sim_p3at/odom is in the odom frame, not the map frame.
- TF already contains map -> odom (from slam_gmapping) and odom -> base_link (from Gazebo),
  so we can publish the fused pose in map.

Parameters:
- ~map_frame (default: map)
- ~base_frame (default: base_link)
- ~out_topic (default: /robot/pose)
- ~rate_hz (default: 30.0)
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped


class TfToPose:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.out_topic = rospy.get_param("~out_topic", "/robot/pose")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(self.out_topic, PoseStamped, queue_size=10)

    def spin(self):
        r = rospy.Rate(max(self.rate_hz, 1.0))
        while not rospy.is_shutdown():
            try:
                tfm = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.2)
                )
                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.map_frame
                msg.pose.position.x = tfm.transform.translation.x
                msg.pose.position.y = tfm.transform.translation.y
                msg.pose.position.z = tfm.transform.translation.z
                msg.pose.orientation = tfm.transform.rotation
                self.pub.publish(msg)
            except Exception as e:
                rospy.logwarn_throttle(
                    2.0,
                    "tf_to_pose: waiting for TF %s -> %s (%s)",
                    self.map_frame,
                    self.base_frame,
                    str(e),
                )
            r.sleep()


def main():
    rospy.init_node("tf_to_pose")
    TfToPose().spin()


if __name__ == "__main__":
    main()
