#! /usr/bin/env python

from cmath import sqrt
import rospy
from nav_msgs.msg import Odometry
from apf.srv import SharePoses2, SharePoses2Response
from tf.transformations import euler_from_quaternion

class PoseService(object):
    def __init__(self, pose_srv_name):
        
        # data
        self.pose_srv_name = pose_srv_name

        # ros
        rospy.on_shutdown(self.shutdown_hook)

        # data
        self.count = 0
        self.ids = []
        self.xt = {}
        self.yt = {}
        self.topics = {}
        self.priorities = {}

        # service
        self.srv = rospy.Service(pose_srv_name, SharePoses2, self.pose_cb)

    def pose_cb(self, req):
        req_i = req.ind
        resp = SharePoses2Response()

        if req.update:
            self.update_goal(req)
            return resp

        xy = {}
        for i in self.ids:
            x,y, h = self.get_odom(self.topics[i])
            xy[i] = [x, y]

            if i==req_i:
                continue
            resp.x.append(x)
            resp.y.append(y)
            resp.heading.append(h)
        
        priorities = self.cal_priorities(xy, req_i)
        resp.priority = priorities
        resp.count = self.count-1
        return resp
    
    def cal_priorities(self, xy, req_i):
        distances = {}
        for i in self.ids:
            distances[i] = ((self.xt[i]-xy[i][0])**2 + (self.yt[i]-xy[i][1])**2)
        
        distances = dict(sorted(distances.items(), key=lambda item: -item[1]))
        del distances[req_i]
        
        priorities = list(distances.keys())
        return priorities

    

    def update_goal(self, req):
        self.xt[req.ind] = req.xt
        self.yt[req.ind] = req.yt


    def get_odom(self, topic):
        topic_msg = None
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(topic, Odometry, timeout=3.0)
            except:
                rospy.loginfo(", current topic is not ready yet, retrying ...")
        odom = topic_msg
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        x = position.x
        y = position.y
        heading = orientation[2]
        return x, y, heading


    def shutdown_hook(self):
        print("shutting down from pose service")
