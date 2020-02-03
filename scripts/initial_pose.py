#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitPose:
    def __init__(self):
        rospy.init_node('initial_pose')
        self.init_pose_pub = rospy.Publisher('initial_pose', PoseWithCovarianceStamped, queue_size = 0)
        self.initialpose = PoseWithCovarianceStamped()
        self.initialpose.header.seq = 0
        self.initialpose.header.stamp = rospy.Time.now()
        self.initialpose.header.frame_id = "odom"
        self.rate = rospy.Rate(1)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        it = InitPose()
        it.init_pose_pub.publish(it.initialpose)
        it.rate.sleep()