#!/usr/bin/env python3
"""
Relay RGB camera_info to a depth camera_info topic.
Legacy workaround if the depth camera_info is missing.
"""
import rospy
from sensor_msgs.msg import CameraInfo

class CameraInfoRelay:
    def __init__(self):
        rospy.init_node('camera_info_relay', anonymous=False)
        
        # Subscribe to RGB camera_info
        self.sub = rospy.Subscriber(
            '/sim_p3at/camera/color/camera_info',
            CameraInfo,
            self.callback,
            queue_size=10
        )
        
        # Publish to depth camera_info
        self.pub = rospy.Publisher(
            '/sim_p3at/camera/depth/camera_info',
            CameraInfo,
            queue_size=10
        )
        
        rospy.loginfo("Camera info relay started: rgb -> depth")
    
    def callback(self, msg):
        # Simply republish the same message
        self.pub.publish(msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        relay = CameraInfoRelay()
        relay.run()
    except rospy.ROSInterruptException:
        pass