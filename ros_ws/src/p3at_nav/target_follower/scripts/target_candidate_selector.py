#!/usr/bin/env python3
"""Select the easiest target from YOLO-style candidates.

Selection rule (Option A): minimise (distance_to_robot / confidence).

Inputs:
  ~candidates_topic (hcr_msgs/TargetCandidateArray)
Output:
  ~selected_topic (geometry_msgs/PointStamped)

Notes:
  - Candidate poses are assumed to be expressed in the frame given by the array header.
  - Candidates are transformed into ~map_frame using TF2 before scoring.
"""

import math
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs

from hcr_msgs.msg import TargetCandidateArray


def euclidean_distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def main():
    rospy.init_node('target_candidate_selector')

    candidates_topic = rospy.get_param('~candidates_topic', '/targets/candidates')
    selected_topic = rospy.get_param('~selected_topic', '/targets/selected_pos')
    map_frame = rospy.get_param('~map_frame', 'map')
    base_frame = rospy.get_param('~base_frame', 'base_link')
    min_confidence = float(rospy.get_param('~min_confidence', 0.05))
    max_age_sec = float(rospy.get_param('~max_age_sec', 1.0))

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher(selected_topic, PointStamped, queue_size=10)

    state = {'last_msg': None, 'last_rx': rospy.Time(0)}

    def cb(msg: TargetCandidateArray):
        state['last_msg'] = msg
        state['last_rx'] = rospy.Time.now()

    rospy.Subscriber(candidates_topic, TargetCandidateArray, cb, queue_size=1)

    rate_hz = float(rospy.get_param('~rate_hz', 10.0))
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("[target_candidate_selector] Subscribed to %s, publishing %s", candidates_topic, selected_topic)

    while not rospy.is_shutdown():
        msg = state['last_msg']
        if msg is None:
            rate.sleep()
            continue

        if (rospy.Time.now() - state['last_rx']).to_sec() > max_age_sec:
            # stale input
            rate.sleep()
            continue

        # get robot position in map frame
        try:
            tf_map_base = tf_buffer.lookup_transform(map_frame, base_frame, rospy.Time(0), rospy.Duration(0.2))
            robot_xyz = (
                tf_map_base.transform.translation.x,
                tf_map_base.transform.translation.y,
                tf_map_base.transform.translation.z,
            )
        except Exception as ex:
            rospy.logwarn_throttle(2.0, "[target_candidate_selector] TF lookup %s->%s failed: %s", map_frame, base_frame, str(ex))
            rate.sleep()
            continue

        src_frame = msg.header.frame_id if msg.header.frame_id else map_frame

        best = None
        best_score = None

        for c in msg.candidates:
            conf = float(c.confidence)
            if conf < min_confidence:
                continue

            pose_stamped = PoseStamped()
            pose_stamped.header = Header(stamp=msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now(),
                                         frame_id=src_frame)
            pose_stamped.pose = c.pose

            # transform candidate into map frame
            try:
                if src_frame != map_frame:
                    pose_map = tf_buffer.transform(pose_stamped, map_frame, rospy.Duration(0.2))
                else:
                    pose_map = pose_stamped
            except Exception as ex:
                rospy.logwarn_throttle(2.0, "[target_candidate_selector] TF transform %s->%s failed: %s", src_frame, map_frame, str(ex))
                continue

            cand_xyz = (
                pose_map.pose.position.x,
                pose_map.pose.position.y,
                pose_map.pose.position.z,
            )

            dist = euclidean_distance(robot_xyz, cand_xyz)
            score = dist / max(conf, min_confidence)

            if best_score is None or score < best_score:
                best_score = score
                best = (c.id, c.class_name, conf, cand_xyz)

        if best is not None:
            out = PointStamped()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = map_frame
            out.point.x = best[3][0]
            out.point.y = best[3][1]
            out.point.z = best[3][2]
            pub.publish(out)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
