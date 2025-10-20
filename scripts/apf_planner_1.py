#! /usr/bin/env python

import numpy as np
from logger import MyLogger
from parameters import Params
from create_model import MRSModel, Robot
from apf.msg import RobotData, FleetData
from apf_planner_base import APFPlannerBase
from my_utils import cal_angle_diff


class APFPlanner(APFPlannerBase):

    def __init__(self, model: MRSModel, robot: Robot, params: Params):
        APFPlannerBase.__init__(self, model, robot, params)
        self.lg = MyLogger(f"APFPlanner1_r{robot.rid}")

    def f_target(self):
        # r_g
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        goal_dist = np.sqrt(dx**2 + dy**2)
        f = self.params.zeta * goal_dist + 0.5
        # f = max(f, self.params.fix_f2)  # change
        # f = self.params.fix_f
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
            dx = self.pose.x - rob.x
            dy = self.pose.y - rob.y
            d_ro = np.sqrt(dx**2 + dy**2)
            theta = np.arctan2(dy, dx)
            angle_diff = cal_angle_diff(theta, self.pose.theta)

            # check distance
            if d_ro > 1 * self.params.robot_start_d:
                continue

            # check if robot should stop
            if (not rob.reached) and (d_ro < self.params.robot_half_d) and (rob.priority > self.robot_data.priority):
                self.stopped = True
                break

            # force
            robot_flag = True
            f = ((self.params.robot_z * 1) * ((1 / d_ro) - (1 / self.params.robot_start_d))**2) * (1 / d_ro)**2
            templ = [f * np.cos(angle_diff), f * np.sin(angle_diff)]
            robot_f[0] += round(templ[0], 3)
            robot_f[1] += round(templ[1], 3)

        # final force
        coeff_f = 1
        if robot_flag:
            self.robot_f[0] += round(robot_f[0] * coeff_f, 3)
            self.robot_f[1] += round(robot_f[1] * coeff_f, 3)

    def f_obstacle(self):
        obst_flag = False
        self.obs_f = [0, 0]
        obs_f = [0, 0]
        for obst in self.obstacles:
            dy = self.pose.y - obst.y
            dx = self.pose.x - obst.x
            d_ro = np.sqrt(dx**2 + dy**2)

            # check distance
            if d_ro > obst.obst_start_d:
                continue

            obst_flag = True
            theta = np.arctan2(dy, dx)
            angle_diff = cal_angle_diff(theta, self.pose.theta)

            # force
            f = ((obst.obst_z * 1) * ((1 / d_ro) - (1 / obst.obst_start_d))**2) * (1 / d_ro)**2
            templ = [f * np.cos(angle_diff), f * np.sin(angle_diff)]
            obs_f[0] += round(templ[0], 3)
            obs_f[1] += round(templ[1], 3)

        # final force
        coeff_f = 1
        if obst_flag:
            self.obs_f[0] += round(obs_f[0] * coeff_f, 3)
            self.obs_f[1] += round(obs_f[1] * coeff_f, 3)
