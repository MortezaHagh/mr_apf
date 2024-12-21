#! /usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from tf.transformations import euler_from_quaternion
from parameters import Params
from create_model import MRSModel
from apf_planner_2 import APFPlanner
from visualization import RvizViusalizer
from mrapf_classes import PRobot, PlannerData, AllRobotsData
from robot_planner import RobotPlanner


class Planner2D (RobotPlanner):
    ard: AllRobotsData

    def __init__(self, model: MRSModel, robot: PRobot, params: Params):
        RobotPlanner.__init__(self, model, robot, params)

        #
        self.do_viz = False
        self.v = 0
        self.w = 0
        self.rid = params.id
        self.ns = params.ns  # name space

        # model
        self.model = model

        # apf planner
        self.ap = APFPlanner(model, robot, params)

        #
        self.pose = Pose2D()        # robot pose
        self.pd = PlannerData()        # records
        self.ard = AllRobotsData()  # all robots' data

        # ros
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown_hook)

        # RvizViusalizer
        self.vs = RvizViusalizer(model)

    def initiate_robots(self):
        # pose = [self.model.robots[i].xs, self.model.robots[i].ys, self.model.robots[i].heading]
        x = self.model.robots[i].xs
        self.ard.update()

    def go_to_goal(self):
        while (not self.ap.reached) and (not rospy.is_shutdown()):

            # record path
            self.pd.x.append(round(self.pose.x, 3))
            self.pd.y.append(round(self.pose.y, 3))

            # get all robots' data
            req_poses = SharePoses2Request()
            req_poses.id = self.rid
            req_poses.update = False
            req_poses.stopped = False
            self.ard.update_by_resp(self.pose_client(req_poses))

            # command ************************************
            self.ap.planner_move(self.pose, self.ard)
            self.v = self.ap.v
            self.w = self.ap.w
            self.pd.v.append(self.v)
            self.pd.w.append(self.w)

            # vizualize
            if self.do_viz:
                self.vs.robot_poly([[], self.ap.mp_bound], self.ns)
                self.vs.draw_robot_circles(self.ap.multi_robots_vis, self.ns)
                self.vizualize_force([self.ap.f_r, self.ap.f_theta], False)

            # publish cmd
            move_cmd = Twist()
            move_cmd.linear.x = self.v
            move_cmd.angular.z = self.w
            self.cmd_vel_pub.publish(move_cmd)

            # check stop
            if self.ap.stopped:
                req = SharePoses2Request()
                req.id = self.rid
                req.stopped = True
                self.pose_client(req)

            # log data
            self.log_motion(self.ap.f_r, self.ap.f_theta)
            self.rate.sleep()

            #
            if self.ap.reached:
                rospy.loginfo(f"[planner_ros, {self.ns}]: robot reached goal.")

        # reached
        req = SharePoses2Request()
        req.id = self.rid
        req.reached = True
        self.pose_client(req)
        self.stop()
        self.stop()
        rospy.loginfo(f"[planner_ros, {self.ns}]: go_to_goal finished!")
