#!/usr/bin/env python3
"""Mux dynamic target position.

Publishes a single target position on ~output_topic.

If a real target on ~real_topic is received recently, forward it.
Otherwise publish a slow, deterministic fake target in the map frame so that
the follower can be tested end-to-end in an empty/mostly-static Gazebo scene.
"""

import math
import rospy
from geometry_msgs.msg import PointStamped


class TargetPosMux:
    def __init__(self):
        self.real_topic = rospy.get_param("~real_topic", "/dynamic_map/dynamic_pos")
        self.output_topic = rospy.get_param("~output_topic", "/dynamic_map/target_pos")
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.real_timeout = float(rospy.get_param("~real_timeout", 1.0))
        self.pub_rate_hz = float(rospy.get_param("~pub_rate_hz", 10.0))

        # Fake target motion (circle)
        self.fake_center_x = float(rospy.get_param("~fake_center_x", 5.0))
        self.fake_center_y = float(rospy.get_param("~fake_center_y", 0.0))
        self.fake_radius = float(rospy.get_param("~fake_radius", 2.0))
        self.fake_omega = float(rospy.get_param("~fake_omega", 0.15))  # rad/s

        self.last_real = None
        self.last_real_time = rospy.Time(0)

        self.pub = rospy.Publisher(self.output_topic, PointStamped, queue_size=10)
        rospy.Subscriber(self.real_topic, PointStamped, self._cb_real, queue_size=10)

        self.t0 = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.pub_rate_hz, 0.1)), self._on_timer)

        rospy.loginfo("target_pos_mux: real_topic=%s -> output_topic=%s", self.real_topic, self.output_topic)

    def _cb_real(self, msg: PointStamped):
        self.last_real = msg
        self.last_real_time = rospy.Time.now()

    def _fake_target(self, stamp: rospy.Time) -> PointStamped:
        t = (stamp - self.t0).to_sec()
        ang = self.fake_omega * t
        p = PointStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.map_frame
        p.point.x = self.fake_center_x + self.fake_radius * math.cos(ang)
        p.point.y = self.fake_center_y + self.fake_radius * math.sin(ang)
        p.point.z = 0.0
        return p

    def _on_timer(self, _evt):
        now = rospy.Time.now()
        if self.last_real is not None and (now - self.last_real_time).to_sec() <= self.real_timeout:
            out = PointStamped()
            out.header.stamp = now
            out.header.frame_id = self.map_frame
            out.point = self.last_real.point
        else:
            out = self._fake_target(now)
        self.pub.publish(out)


def main():
    rospy.init_node("target_pos_mux")
    TargetPosMux()
    rospy.spin()


if __name__ == "__main__":
    main()
