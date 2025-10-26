#! /usr/bin/env python
""" subscribes to /r#/odom and publishes tf /odom - /r#/odom"""

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from create_model import MRSModel, Robot
from parameters import Params


class StaticBroadCaster:
    def __init__(self):
        self.ns = 'StaticBroadCaster'
        self.params = Params()
        self.model = MRSModel(params=self.params)
        #
        self.transforms = []
        self.broadcasters = []

    def all_static_transforms(self):
        self.static_transforms(ns="", x=0, y=0, heading=0)
        #
        robo: Robot = None
        for robo in self.model.robots:
            x = robo.xs
            y = robo.ys
            heading = robo.heading
            ns = robo.ns
            self.static_transforms(ns, x, y, heading)

    def static_transforms(self, ns: str, x, y, heading):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = '/map'
        t.child_frame_id = ns + "/odom"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.transforms.append(t)
        # send transform
        stb = StaticTransformBroadcaster()
        stb.sendTransform(t)
        self.broadcasters.append(stb)
        rospy.loginfo(f"[{self.ns}]: Broadcasting static transform for {t.child_frame_id} at ({x}, {y})")


if __name__ == "__main__":
    rospy.init_node("broadcasters")
    broadcaster = StaticBroadCaster()
    broadcaster.all_static_transforms()
    rospy.spin()
