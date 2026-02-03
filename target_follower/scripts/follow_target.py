#!/usr/bin/env python3
import math
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TargetFollower:
    def __init__(self):
        self.target_topic = rospy.get_param("~target_topic", "/dynamic_map/dynamic_pos")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.follow_distance = float(rospy.get_param("~follow_distance", 1.5))
        self.goal_rate_hz = float(rospy.get_param("~goal_rate_hz", 2.0))
        self.target_timeout = float(rospy.get_param("~target_timeout", 1.0))
        self.max_goal_step = float(rospy.get_param("~max_goal_step", 0.8))  # limit jumpiness

        self.last_target = None
        self.last_target_time = rospy.Time(0)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base. Subscribing to %s", self.target_topic)

        rospy.Subscriber(self.target_topic, PointStamped, self.cb_target, queue_size=10)

        self.prev_goal_xy = None
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.goal_rate_hz, 0.1)), self.on_timer)

    def cb_target(self, msg):
        self.last_target = msg
        self.last_target_time = rospy.Time.now()

    def get_robot_xy(self):
        try:
            tfm = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.2))
            x = tfm.transform.translation.x
            y = tfm.transform.translation.y
            return x, y
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF lookup failed (%s -> %s): %s", self.map_frame, self.base_frame, str(e))
            return None

    @staticmethod
    def clamp_step(prev, nxt, max_step):
        if prev is None:
            return nxt
        dx = nxt[0] - prev[0]
        dy = nxt[1] - prev[1]
        d = math.hypot(dx, dy)
        if d <= max_step:
            return nxt
        s = max_step / max(d, 1e-6)
        return (prev[0] + s * dx, prev[1] + s * dy)

    def on_timer(self, _event):
        now = rospy.Time.now()
        if self.last_target is None or (now - self.last_target_time).to_sec() > self.target_timeout:
            # No target. Keep the current goal running. Optionally you could cancel here.
            rospy.logwarn_throttle(2.0, "No recent target on %s", self.target_topic)
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return

        # Use target coordinates as-is (assumed in map frame). If your target is not in map frame,
        # publish it in map frame first or add a TF transform here.
        tx = self.last_target.point.x
        ty = self.last_target.point.y

        rx, ry = robot_xy
        vx = tx - rx
        vy = ty - ry
        dist = math.hypot(vx, vy)

        if dist < 1e-3:
            return

        # Goal is behind target along the robot-target line, keeping follow_distance
        ux = vx / dist
        uy = vy / dist
        gx = tx - ux * self.follow_distance
        gy = ty - uy * self.follow_distance

        # Limit goal jumps to keep the local planner stable
        gx, gy = self.clamp_step(self.prev_goal_xy, (gx, gy), self.max_goal_step)
        self.prev_goal_xy = (gx, gy)

        yaw = math.atan2(uy, ux)  # face toward target
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = now
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        self.client.send_goal(goal)

def main():
    rospy.init_node("target_follower")
    TargetFollower()
    rospy.spin()

if __name__ == "__main__":
    main()
