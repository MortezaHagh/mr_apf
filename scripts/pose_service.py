#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from apf.srv import SharePoses, SharePosesResponse
# from tf.transformations import euler_from_quaternion

class PoseService(object):
    def __init__(self, pose_srv_name):
        
        # data
        self.pose_srv_name = pose_srv_name

        # ros
        rospy.on_shutdown(self.shutdown_hook)

        # data
        self.ids = []
        self.xt = []
        self.yt = []
        self.count = 0
        self.topics = []
        self.priorities = []

        # service
        self.srv = rospy.Service(pose_srv_name, SharePoses, self.pose_cb)

    def pose_cb(self, req):
        req_i = req.ind
        resp = SharePosesResponse()
        
        inds = [i for i, id in enumerate(self.ids) if id!=req_i]
        for i in inds:
            x,y = self.get_odom(self.topics[i])
            resp.x.append(x)
            resp.y.append(y)
            resp.priority.append(self.priorities[i])
      
        resp.count = self.count-1
        return resp


    def get_odom(self, topic):
        topic_msg = None
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(topic, Odometry, timeout=3.0)
            except:
                rospy.loginfo(", current topic is not ready yet, retrying ...")
        odom = topic_msg
        position = odom.pose.pose.position
        x = position.x
        y = position.y
        # quaternion = odom.pose.pose.orientation
        # orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        # orientation = orientation[2]
        # heading = round(orientation, 2)
        return x, y


    def shutdown_hook(self):
        print("shutting down from pose service")
