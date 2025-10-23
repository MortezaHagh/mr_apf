#! /usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from parameters import Params
from mrapf_classes import PlannerData
from visualization import RvizViusalizer
from create_model import MRSModel, Robot
from apf_planner_base import APFPlannerBase
from apf_planner_1 import APFPlanner as APFPlanner1
from apf_planner_2 import APFPlanner as APFPlanner2
from apf_planner_3 import APFPlanner as APFPlanner3
from apf_planner_x import APFPlanner as APFPlannerX
from apf.srv import SendRobotUpdate, SendRobotUpdateRequest
from apf.msg import FleetData


class RobotPlannerBase:

    def __init__(self, model: MRSModel, robot: Robot, params: Params):

        #
        self.v = 0
        self.w = 0
        self.do_viz: bool = True
        self.is_reached = False
        self.abort: bool = False

        #
        self.rid: int = robot.rid
        self.ns: str = robot.ns  # name space

        # apf planner
        self.params: Params = params
        if params.method == 1:
            self.mrapf = APFPlanner1(model, robot, params)
        elif params.method == 2:
            self.mrapf = APFPlanner2(model, robot, params)
        elif params.method == 3:
            self.mrapf = APFPlanner3(model, robot, params)
        else:
            self.mrapf = APFPlannerX(model, robot, params)
            # raise ValueError("method not defined")

        #
        self.pose = Pose2D()        # robot pose
        self.pd = PlannerData()     # planner data
        self.fleet_data = FleetData()

        # ros
        f = 10
        self.dt = 1/f
        self.rate = rospy.Rate(f)
        rospy.on_shutdown(self.shutdown_hook)

        # RvizViusalizer for forces (arrow)
        self.viusalizer = RvizViusalizer(model)

        # listener
        self.data_received: bool = False
        rospy.Subscriber(params.fleet_data_topic, FleetData, self.fleet_data_cb, queue_size=2)

        # /cmd_vel puplisher
        self.cmd_vel_pub = rospy.Publisher(params.cmd_topic, Twist, queue_size=5)

        # Send Robot Update - target
        rospy.wait_for_service(params.sru_srv_name)
        self.robot_update_client = rospy.ServiceProxy(params.sru_srv_name, SendRobotUpdate)
        self.ruc_req = SendRobotUpdateRequest()
        self.ruc_req.rid = self.rid
        self.ruc_req.xt = robot.xt
        self.ruc_req.yt = robot.yt
        self.ruc_req.stopped = False
        self.ruc_req.reached = False
        self.ruc_req.stuck = False

    def start_planner(self):
        # start time
        self.pd.set_start_time(rospy.get_time())

        self.is_reached = self.go_to_goal()
        self.after_reached()

        # end time
        self.pd.set_success(self.is_reached)
        self.pd.set_end_time(rospy.get_time())
        self.pd.finalize()

    def go_to_goal(self):
        pass
        # while (not self.mrapf.reached) and (not rospy.is_shutdown()):
        #     ...
        #     self.next_move()

    def next_move(self):
        self.mrapf.planner_next_move(self.pose, self.fleet_data)
        self.v = self.mrapf.v
        self.w = self.mrapf.w

        # update planner data
        self.pd.append_data(self.pose.x, self.pose.y, self.v, self.w)

        # vizualize
        if self.do_viz:
            self.vizualize_force([self.mrapf.f_r, self.mrapf.f_theta], False)
            if self.params.method > 1:
                self.viusalizer.draw_fake_obsts_localmin(self.mrapf.fake_obsts_localmin)
                self.viusalizer.visualize_polygon(self.mrapf.clusters_x_y, self.ns, self.params.rid)
                self.viusalizer.draw_robot_circles(self.mrapf.multi_robots_vis, self.ns)

        # publish cmd
        move_cmd = Twist()
        move_cmd.linear.x = self.v
        move_cmd.angular.z = self.w
        self.cmd_vel_pub.publish(move_cmd)

        # check stop
        if self.mrapf.stopped != self.mrapf.prev_stopped:
            self.ruc_req.stopped = self.mrapf.stopped
            self.robot_update_client(self.ruc_req)
            self.mrapf.prev_stopped = self.mrapf.stopped

        # check progress
        if self.mrapf.stuck != self.mrapf.prev_stuck:
            self.ruc_req.stuck = self.mrapf.stuck
            self.robot_update_client(self.ruc_req)
            self.mrapf.prev_stuck = self.mrapf.stuck

        # log data
        self.log_motion(self.mrapf.f_r, self.mrapf.f_theta)

        # reached
        if self.mrapf.reached:
            rospy.loginfo(f"[planner_base, {self.ns}]: robot reached goal.")

    def after_reached(self):
        self.stop()
        self.ruc_req.stopped = True
        self.ruc_req.reached = True
        self.robot_update_client(self.ruc_req)
        rospy.loginfo(f"[planner_base, {self.ns}]: go_to_goal finished!")

    def stop(self):
        t = 0
        while t < 4:
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
            # rospy.loginfo(f"[planner_base, {self.ns}]: {self.mrapf.stop_flag_multi}")
            # rospy.loginfo(f"[planner_base, {self.ns}]: f_r: {fr}, f_theta: {ft}")
            # rospy.loginfo(f"[planner_base, {self.ns}]: v: {v}, w: {w}")
            # rospy.loginfo(" --------------------------------------------------- ")

    def fleet_data_cb(self, msg: FleetData):
        if msg.success:
            self.data_received = True
        else:
            self.data_received = False
        self.fleet_data = msg

    def stop_planner(self):
        rospy.loginfo(f"[planner_base, {self.ns}]: stopping planner ... ")
        self.abort = True
        self.stop()

    def shutdown_hook(self):
        rospy.loginfo(f"[planner_base, {self.ns}]: shutting ...")
        self.stop()

    def vizualize_force(self, nr_force, ip=True):
        theta = np.arctan2(nr_force[1], nr_force[0])
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        theta = self.pose.theta + theta
        if ip:
            self.viusalizer.visualize_arrow(self.pose.x, self.pose.y, theta)
        else:
            self.viusalizer.visualize_arrow(self.pose.x, self.pose.y, theta, self.ns)
