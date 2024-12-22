#! /usr/bin/env python

import numpy as np
from parameters import Params
from geometry_msgs.msg import Pose2D
from create_model import MRSModel, Robot
from apf.msg import RobotData, FleetData
from apf_planner_base import APFPlannerBase
from my_utils import cal_angle_diff


class APFPlanner(APFPlannerBase):

    def __init__(self, model: MRSModel, robot: Robot, params: Params):
        APFPlannerBase.__init__(self, model, robot, params)

    def planner_move(self, pose: Pose2D, fleet_data: FleetData):
        # inputs - reset
        self.pose = pose
        self.fleet_data = fleet_data
        self.reset_vals()

        # get this robot data
        crob: RobotData = None
        for crob in fleet_data.fdata:
            if crob.rid == self.robot.rid:
                self.pose.x = crob.x
                self.pose.y = crob.y
                self.pose.theta = crob.h
                # rd = crob
                break

        # check dist to goal
        if self.goal_dist < self.p.goal_dis_tresh:
            self.reached = True
            return

        # calculate forces =================================================
        self.forces()

        # calculate velocities
        self.cal_vel()

        # check stop_flag_full
        if self.stopped:
            print("[planner_move, {self.robot.rid}], stop_flag_full")
            self.v = 0
            # self.w = 0

    def f_target(self):
        # r_g
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        goal_dist = np.sqrt(dx**2 + dy**2)
        f = self.p.zeta * goal_dist
        f = max(f, self.p.fix_f2)  # change
        # f = self.p.fix_f
        theta_rg = np.arctan2(dy, dx)
        ad_rg_h = cal_angle_diff(theta_rg, self.pose.theta)
        self.ad_rg_h = ad_rg_h
        self.theta_rg = theta_rg
        self.goal_dist = goal_dist
        self.goal_theta = theta_rg
        fx = round(f * np.cos(ad_rg_h), 3)
        fy = round(f * np.sin(ad_rg_h), 3)
        self.target_f = [fx, fy]

    def f_robots(self):
        #
        robot_flag = False
        fd: FleetData = self.fleet_data
        #
        robot_f = [0, 0]
        self.robot_f = [0, 0]
        #
        rob: RobotData
        for rob in fd.fdata:
            if rob.rid == self.robot.rid:
                continue
            dx = -(rob.x - self.pose.x)
            dy = -(rob.y - self.pose.y)
            d_ro = np.sqrt(dx**2 + dy**2)
            theta = np.arctan2(dy, dx)
            angle_diff = cal_angle_diff(theta, self.pose.theta)

            if d_ro > 1 * self.p.robot_start_d:
                continue

            # and abs(angle_diff) > np.pi/2:
            if (not rob.reached) and d_ro < self.p.obst_half_d and rob.priority > 0:
                self.stopped = True
                break

            robot_flag = True
            f = ((self.p.robot_z * 1) * ((1 / d_ro) -
                 (1 / self.p.robot_start_d))**2) * (1 / d_ro)**2
            templ = [f * np.cos(angle_diff), f * np.sin(angle_diff)]

            robot_f[0] += round(templ[0], 3)
            robot_f[1] += round(templ[1], 3)

        coeff_f = 1
        if robot_flag:
            self.robot_f[0] += round(robot_f[0] * coeff_f, 3)
            self.robot_f[1] += round(robot_f[1] * coeff_f, 3)

    def f_obstacle(self):
        obst_flag = False
        self.obs_f = [0, 0]
        obs_f = [0, 0]
        for i in range(self.obs_count):
            dy = -(self.obs_y[i] - self.pose.y)
            dx = -(self.obs_x[i] - self.pose.x)
            d_ro = np.sqrt(dx**2 + dy**2)

            if d_ro > self.p.obst_start_d:
                continue

            obst_flag = True
            theta = np.arctan2(dy, dx)
            angle_diff = theta - self.pose.theta
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

            f = ((self.p.obst_z * 1) * ((1 / d_ro) -
                 (1 / self.p.obst_start_d))**2) * (1 / d_ro)**2
            templ = [f * np.cos(angle_diff), f * np.sin(angle_diff)]

            obs_f[0] += round(templ[0], 3)
            obs_f[1] += round(templ[1], 3)

        coeff_f = 1
        if obst_flag:
            self.obs_f[0] += round(obs_f[0] * coeff_f, 3)
            self.obs_f[1] += round(obs_f[1] * coeff_f, 3)
