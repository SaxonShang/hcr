#!/usr/bin/env python3
"""Publish a YOLO-like TargetCandidateArray using Gazebo model ground truth.

This is for simulation only.

It reads /gazebo/model_states, extracts the pose for a named model (default: target_box),
and publishes hcr_msgs/TargetCandidateArray on /targets/candidates.

Frame conventions:
  - Gazebo model poses are expressed in the Gazebo world frame.
  - In this project, the robot odom frame is initialized at the world origin.
    Publishing the candidates in frame_id="odom" keeps the target consistent with the
    navigation TF tree (map<->odom<->base_link).

Parameters:
  ~model_name (str)
  ~frame_id (str)
  ~candidates_topic (str)
  ~id (int)
  ~class_name (str)
  ~confidence (float)
  ~rate_hz (float)
"""

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header

from hcr_msgs.msg import TargetCandidate, TargetCandidateArray


class MockYoloFromGazebo:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "target_box")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.candidates_topic = rospy.get_param("~candidates_topic", "/targets/candidates")

        self.cand_id = int(rospy.get_param("~id", 1))
        self.class_name = rospy.get_param("~class_name", "target_box")
        self.confidence = float(rospy.get_param("~confidence", 0.9))
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))

        self._last_pose = None
        self._last_stamp = rospy.Time(0)

        self.pub = rospy.Publisher(self.candidates_topic, TargetCandidateArray, queue_size=10)
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb, queue_size=1)

        rospy.loginfo(
            "[mock_yolo_from_gazebo] model=%s -> %s (frame_id=%s)",
            self.model_name,
            self.candidates_topic,
            self.frame_id,
        )

    def _cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(2.0, "[mock_yolo_from_gazebo] model '%s' not found in /gazebo/model_states", self.model_name)
            return

        self._last_pose = msg.pose[idx]
        self._last_stamp = rospy.Time.now()

    def spin(self):
        r = rospy.Rate(max(self.rate_hz, 0.1))
        while not rospy.is_shutdown():
            if self._last_pose is None:
                r.sleep()
                continue

            out = TargetCandidateArray()
            out.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)

            c = TargetCandidate()
            c.id = self.cand_id
            c.class_name = self.class_name
            c.confidence = self.confidence
            c.pose = self._last_pose

            out.candidates = [c]
            self.pub.publish(out)

            r.sleep()


def main():
    rospy.init_node("mock_yolo_from_gazebo")
    node = MockYoloFromGazebo()
    node.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
