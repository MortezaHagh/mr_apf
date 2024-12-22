#! /usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from parameters import Params
from mrapf_classes import PlannerData
from apf_planner_2 import APFPlanner
from visualization import RvizViusalizer
from create_model import MRSModel, Robot
from apf.msg import FleetData
# choose: [apf_planner_2], [apf_planner_1]


class RobotPlanner:
    do_viz: bool
    rid: int
    ns: str
    p: Params
    ap: APFPlanner
    vs: RvizViusalizer
    pose: Pose2D
    pd: PlannerData  # planner data
    fleet_data: FleetData

    def __init__(self, model: MRSModel, robot: Robot, params: Params):

        #
        self.v = 0
        self.w = 0
        self.do_viz = True
        self.is_reached = False

        #
        self.rid = robot.rid
        self.ns = robot.ns  # name space

        # apf planner
        self.ap = APFPlanner(model, robot, params)

        #
        self.pose = Pose2D()        # robot pose
        self.pd = PlannerData()     # planner data
        self.fleet_data = FleetData()

        # ros
        f = 10
        self.dt = 1/f
        self.rate = rospy.Rate(f)
        rospy.on_shutdown(self.shutdown_hook)

        # RvizViusalizer
        self.vs = RvizViusalizer(model)

        # /cmd_vel puplisher
        self.cmd_vel_pub = rospy.Publisher(params.cmd_topic, Twist, queue_size=5)

    def start(self):
        # start time
        self.pd.start_t = rospy.get_time()

        self.go_to_goal()
        self.is_reached = True

        # end time
        self.pd.end_t = rospy.get_time()
        self.pd.finalize()

    def go_to_goal(self):
        pass

    def stop(self):
        t = 0
        while t < 5:
            self.cmd_vel_pub.publish(Twist())
            self.rate.sleep()
            t += 1

    def log_motion(self, f_r, f_theta):
        n = 1
        if self.rid == n:
            v = round(self.v, 2)
            w = round(self.w, 2)
            fr = round(f_r, 2)
            ft = round(f_theta, 2)
            # rospy.loginfo(f"[planner_base, {self.ns}]: {self.ap.stop_flag_multi}")
            rospy.loginfo(f"[planner_base, {self.ns}]: f_r: {fr}, f_theta: {ft}")
            rospy.loginfo(f"[planner_base, {self.ns}]: v: {v}, w: {w}")
            rospy.loginfo(" --------------------------------------------------- ")

    def shutdown_hook(self):
        rospy.loginfo(f"[planner_base, {self.ns}]: shutting ...")
        self.stop()

    def vizualize_force(self, nr_force, ip=True):
        theta = np.arctan2(nr_force[1], nr_force[0])
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        theta = self.pose.theta + theta
        if ip:
            self.vs.arrow(self.pose.x, self.pose.y, theta)
        else:
            self.vs.arrow(self.pose.x, self.pose.y, theta, self.ns)
