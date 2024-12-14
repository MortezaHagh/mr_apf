#! /usr/bin/env python

""" subscribes to /r#/odom and publishes tf /odom - /r#/odom"""

import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class BroadCast:
    def __init__(self, ind):
        self.topic = "/r"+str(ind)+"/odom"
        self.rate = rospy.Rate(10)
        self.tf_static_broadcaster = StaticTransformBroadcaster()

        #
        msg = rospy.wait_for_message(self.topic, Odometry, 2)
        self.pose = msg.pose.pose.position
        self.orien = msg.pose.pose.orientation
        rospy.Subscriber(self.topic, Odometry, self.callback, queue_size=10)

    def callback(self, msg):
        self.pose = msg.pose.pose.position
        self.orien = msg.pose.pose.orientation
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'odom'
        t.child_frame_id = self.topic
        t.transform.translation.x = float(self.pose.x)
        t.transform.translation.y = float(self.pose.y)
        t.transform.translation.z = float(self.pose.z)
        t.transform.rotation.x = self.orien.x
        t.transform.rotation.y = self.orien.y
        t.transform.rotation.z = self.orien.z
        t.transform.rotation.w = self.orien.w
        # send transform
        self.tf_static_broadcaster.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("broadcasters")
    ids = [1, 2, 3, 4]
    for i in ids:
        BroadCast(i)
    rospy.spin()
