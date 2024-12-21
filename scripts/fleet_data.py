#! /usr/bin/env python

from typing import Dict, Tuple, List
import rospy
import tf
from parameters import Params
from apf.msg import RobotData, FleetData
from apf.srv import SendRobotUpdate, SendRobotUpdateRequest, SendRobotUpdateResponse


class FleetDataH:
    nr: int
    ids: List[int]
    ns: Dict[int, str]
    xy: Dict[int, Tuple[float, float]]
    xyt: Dict[int, Tuple[float, float]]
    fleet_data: Dict[int, RobotData]

    def __init__(self, params: Params):

        # data
        self.nr = 0
        self.ids = []
        self.ns = {}
        self.xy = {}
        self.xyt = {}
        self.fleet_data = {}

        # settings
        self.global_frame = params.global_frame
        self.local_frame = params.local_frame

        # tf listener
        self.tf_listener = tf.TransformListener()

        # service
        self.srv = rospy.Service(params.sru_srv_name, SendRobotUpdate, self.sru_cb)

        # publisher
        self.pub = rospy.Publisher(params.fleet_data_topic, FleetData, queue_size=10)

        # # update and pubish data
        # self.update_all()

    def add_robot(self, rid: int, ns: str = ""):
        self.nr += 1
        self.ids.append(rid)
        self.ns[rid] = ns
        self.xy[rid] = (0, 0)
        self.xyt[rid] = (0, 0)
        self.fleet_data[rid] = RobotData()
        self.fleet_data[rid].rid = rid

    def update_goal(self, rid: int, xt: float, yt: float):
        self.xyt[rid] = (xt, yt)

    def sru_cb(self, req: SendRobotUpdateRequest):
        """ Callback for the SendRobotUpdate service """
        rid = req.rid
        self.fleet_data[rid].stopped = req.stopped
        self.fleet_data[rid].reached = req.reached
        resp = SendRobotUpdateResponse()
        resp.success = True
        return resp

    def update_all(self):
        """ Update all robot data """
        fd = FleetData()
        self.get_all_tf()
        self.update_priority()
        # publish msg
        fd.nr = self.nr
        fd.fdata = list(self.fleet_data.values())
        self.pub.publish(fd)

    def get_all_tf(self):
        for rid in self.ids:
            frame = self.ns[rid] + self.local_frame
            self.get_tf(rid, frame)

    def get_tf(self, rid: int, frame: str):
        try:
            self.tf_listener.waitForTransform(self.global_frame, frame, rospy.Time(0), rospy.Duration(0.2))
            (trans, rot) = self.tf_listener.lookupTransform(self.global_frame, frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Exception for robot with frame {frame}: {e}")
            raise
        # Extract the x, y, and yaw from the transformation
        x = trans[0]
        y = trans[1]
        euler = tf.transformations.euler_from_quaternion(rot)
        theta = euler[2]
        #
        self.xy[rid] = (x, y)
        #
        self.fleet_data[rid].x = x
        self.fleet_data[rid].y = y
        self.fleet_data[rid].h = theta

    def update_priority(self):
        dists = []
        max_dist = 0.001
        for rid in self.ids:
            x_t, y_t = self.xyt[rid]
            x_r, y_r = self.fleet_data[rid].x, self.fleet_data[rid].y
            d = ((x_t - x_r) ** 2 + (y_t - y_r) ** 2) ** 0.5
            dists.append(d)
            if d > max_dist:
                max_dist = d
        dists = [d/max_dist for d in dists]
        # update priorities
        for i, rid in enumerate(self.ids):
            self.fleet_data[rid].priority = dists[i]
