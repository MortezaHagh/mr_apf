#! /usr/bin/env python

import rospy
from apf.srv import MyPose, MyPoseResponse


class PoseService(object):
    def __init__(self, poses, count, pose_srv_name):
        
        # ros
        rospy.on_shutdown(self.shutdown_hook)

        # data
        self.count = count
        self.x = [p[0] for p in poses]
        self.y = [p[1] for p in poses]
        self.inds = [i for i in range(count)]

        # service
        self.srv = rospy.Service(pose_srv_name, MyPose, self.pose_cb)

    def pose_cb(self, req):
        req_i = req.ind
        resp = MyPoseResponse()
        inds = [j for j in self.inds if j != req_i]
        resp.x = [self.x[i] for i in inds]
        resp.y = [self.y[i] for i in inds]
        resp.count = self.count-1
        return resp

    def update_poses(self, poses):
        self.x = [p[0] for p in poses]
        self.y = [p[1] for p in poses]

    def shutdown_hook(self):
        print("shutting down from pose service")
