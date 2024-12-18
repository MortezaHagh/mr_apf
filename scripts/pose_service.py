#! /usr/bin/env python

from typing import List, Dict
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from apf.srv import SharePoses2, SharePoses2Response, SharePoses2Request


class PoseService(object):
    xy: Dict[int, List[float]]

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

    def add_robot(self, rid, priority, lis_topic):
        self.count += 1
        self.xt[rid] = 0
        self.yt[rid] = 0
        self.ids.append(rid)
        self.stop[rid] = False
        self.reached[rid] = False
        self.priorities[rid] = priority
        self.topics[rid] = lis_topic

    def pose_cb(self, req: SharePoses2Request):
        req_i = req.id
        resp = SharePoses2Response()

        # update goal
        if req.update:
            self.update_goal(req)
            return resp

        # update reached
        if req.reached:
            self.reached[req_i] = True
            rospy.loginfo("[PoseService]: r{req_i} reached.")
            return resp

        # check if stopped
        if req.stopped:
            self.stop[req_i] = True
            return resp

        # calculate response
        for i in self.ids:
            x, y, h = self.get_odom(self.topics[i])
            self.h[i] = h
            self.xy[i] = [x, y]
            if i == req_i:
                continue
            resp.x.append(x)
            resp.y.append(y)
            resp.h.append(h)
            resp.stopped.append(self.stop[i])
            resp.reached.append(self.reached[i])
        # calculate priorities
        priorities = self.cal_priorities(self.xy, req_i)
        resp.pr = priorities
        resp.nr = self.count-1

        return resp

    def cal_priorities(self, xy, req_i):
        priorities = []
        d0 = (self.xt[req_i]-xy[req_i][0])**2 + (self.yt[req_i]-xy[req_i][1])**2
        for i in self.ids:
            if i == req_i:
                continue
            distance = ((self.xt[i]-xy[i][0])**2 + (self.yt[i]-xy[i][1])**2)
            if distance > d0 and (not self.reached[i]):
                priorities.append(1)
            else:
                priorities.append(-1)
        return priorities

    def update_goal(self, req):
        self.xt[req.id] = req.xt
        self.yt[req.id] = req.yt

    def get_odom(self, topic):
        topic_msg: Odometry = None
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(topic, Odometry, timeout=3.0)
            except rospy.ROSInterruptException as e:
                rospy.logerr(f"[pose_service]: ROS node was interrupted: {e}")
            except rospy.ROSException as e:
                rospy.logerr(f"[pose_service]: Timeout while waiting for message: {e}")
                rospy.loginfo("[pose_service]: current topic is not ready yet, retrying ...")
        odom = topic_msg
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        x = position.x
        y = position.y
        heading = orientation[2]
        return x, y, heading

    def shutdown_hook(self):
        rospy.loginfo("[PoseService]: shutting down from pose service")
