#!/usr/bin/env python3
"""Move the red target box in Gazebo.

This node drives the Gazebo model named "target_box" along a smooth circle so
that the mapping and dynamic-object tracking pipeline has a truly moving target.

It uses the /gazebo/set_model_state service from gazebo_ros.
"""

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


def quaternion_from_yaw(yaw: float):
    """Convert yaw to a quaternion (x,y,z,w)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def main():
    rospy.init_node("move_target_box")

    model_name = rospy.get_param("~model_name", "target_box")
    center_x = float(rospy.get_param("~center_x", 5.0))
    center_y = float(rospy.get_param("~center_y", 0.0))
    radius = float(rospy.get_param("~radius", 1.5))
    omega = float(rospy.get_param("~omega", 0.25))  # rad/s
    z = float(rospy.get_param("~z", 0.75))
    rate_hz = float(rospy.get_param("~rate_hz", 20.0))

    rospy.wait_for_service("/gazebo/set_model_state")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    t0 = rospy.Time.now()
    rate = rospy.Rate(max(rate_hz, 1.0))

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - t0).to_sec()
        ang = omega * t

        x = center_x + radius * math.cos(ang)
        y = center_y + radius * math.sin(ang)
        # Optional: face the direction of motion
        yaw = ang + math.pi / 2.0
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        state = ModelState()
        state.model_name = model_name
        state.reference_frame = "world"

        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        # Keep twist small so physics does not add surprises.
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        try:
            set_state(state)
        except rospy.ServiceException as e:
            rospy.logwarn("move_target_box: set_model_state failed: %s", str(e))

        rate.sleep()


if __name__ == "__main__":
    main()
