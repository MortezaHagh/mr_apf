#! /usr/bin/env python

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
        self.topics = {}
        self.count = 0
        self.ids = []
        self.xt = {}
        self.yt = {}
        self.h = {}
        self.xy = {}
        self.stop = {}
        self.reached = {}
        self.priorities = {}

        # service
        self.srv = rospy.Service(pose_srv_name, SharePoses2, self.pose_cb)

    # ------------------------------- callback

    def pose_cb(self, req):
        req_i = req.ind
        resp = SharePoses2Response()

        if req.update:
            self.update_goal(req)
            return resp

        if req.reached:
            self.reached[req_i] = True
            print("PoseService: ", req_i, "reached")
            return resp
        
        if req.stopped:
            self.stop[req_i] = True
            return resp

        for i in self.ids:
            x,y, h = self.get_odom(self.topics[i])
            self.h[i] = h
            self.xy[i] = [x, y]

            if i==req_i:
                continue
            resp.x.append(x)
            resp.y.append(y)
            resp.heading.append(h)
            resp.stopp.append(self.stop[i])
            resp.reached.append(self.reached[i])
        
        priorities = self.cal_priorities(self.xy, req_i)
        resp.priority = priorities
        resp.count = self.count-1
        
        return resp
    
        # -----------------------------------------------
    
    def cal_priorities(self, xy, req_i):
        priorities = []
        d0 = (self.xt[req_i]-xy[req_i][0])**2 + (self.yt[req_i]-xy[req_i][1])**2
        
        for i in self.ids:
            if i==req_i:
                continue
            distance = ((self.xt[i]-xy[i][0])**2 + (self.yt[i]-xy[i][1])**2)
            if distance>d0 and (not self.reached[i]):
                priorities.append(1)
            else:
                priorities.append(-1)

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
