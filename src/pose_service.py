#! /usr/bin/env python

import rospy
from apf.srv import MyPose, MyPoseResponse


class PoseService(object):
    def __init__(self, poses, count, pose_srv_name):
        self.count = count
        self.rind = [i for i in range(count)]
        self.x = [p[0] for p in poses]
        self.y = [p[1] for p in poses]

        # service
        self.srv = rospy.Service(pose_srv_name, MyPose, self.pose_cb)

    def pose_cb(self, req):
        req_i = req.ind
        resp = MyPoseResponse()
        inds = [j for j in self.rind if j != req_i]
        resp.x = [self.x[i] for i in inds]
        resp.y = [self.y[i] for i in inds]
        resp.count = self.count #mh
        return resp

    def update_poses(self, poses):
        self.x = [p[0] for p in poses]
        self.y = [p[1] for p in poses]
