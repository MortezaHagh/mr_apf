#! /usr/bin/env python

from typing import List
import numpy as np
from geometry_msgs.msg import Pose2D
from parameters import Params
from create_model import MRSModel
from apf.msg import RobotData, FleetData
from mrapf_classes import PRobot, ApfRobot
from my_utils import cal_angle_diff


class APFPlanner:
    p: Params
    model: MRSModel
    robot: PRobot
    pose: Pose2D
    rd: RobotData
    fleet_data: FleetData
    multi_robots_vis: List[ApfRobot]

    def __init__(self, model: MRSModel, robot: PRobot, params: Params):
        # data
        self.p = params
        self.model = model
        self.robot = robot
        self.map_data()

        #
        self.pose = Pose2D()
        self.rd = None
        self.fleet_data = None
        self.multi_robots_vis = []
        self.mp_bound = []

        #
        self.v = 0
        self.w = 0

        #
        self.f_r = 0
        self.f_theta = 0
        self.phi = 0

        # compute vars
        self.ad_rg_h = None
        self.theta_rg = None
        self.goal_dist = None
        self.goal_theta = None
        #
        self.robot_f = []
        self.target_f = []
        self.obs_f = []

        # control vars
        self.reached = False
        self.stopped = False

        # control vars
        self.theta_rg = 0
        self.goal_theta = 0
        self.goal_dist = 1000

    def reset_vals(self):
        self.multi_robots_vis = []
        #
        self.v = 0
        self.w = 0
        #
        self.f_r = 0
        self.f_theta = 0
        self.phi = 0
        #
        self.reached = False
        self.stopped = False

    def planner_move(self, pose: Pose2D, fleet_data: FleetData):
        # inputs - reset
        self.pose = pose
        self.fleet_data = fleet_data
        self.reset_vals()

        # get this robot data
        crob: RobotData = None
        for crob in fleet_data.fdata:
            if crob.rid == self.p.rid:
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
            print("[planner_move, {self.p.rid}], stop_flag_full")
            self.v = 0
            # self.w = 0

    def forces(self):
        # target
        f_r = 0
        f_theta = 0
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]
        # obstacles
        self.f_obstacle()
        f_r += self.obs_f[0]
        f_theta += self.obs_f[1]
        # robots
        self.f_robots()
        f_r += self.robot_f[0]
        f_theta += self.robot_f[1]
        # phi
        theta = np.arctan2(f_theta, f_r)
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        phi = round(theta, 4)

        # update class values
        self.f_r = f_r
        self.f_theta = f_theta
        self.phi = phi

        # self.pd.phis.append(phi)
        # self.pd.f_or.append(self.obs_f[0])
        # self.pd.f_ot.append(self.obs_f[1])
        # self.pd.f_tr.append(self.target_f[0])
        # self.pd.f_tt.append(self.target_f[1])

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
            if rob.rid == self.p.rid:
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

    def map_data(self):
        # robot target
        self.goal_x = self.robot.xt
        self.goal_y = self.robot.yt
        # obstacles
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y
        self.obs_count = self.model.obst.count
        self.obs_ind_main = [i for i in range(self.model.obst.count)]

    def cal_vel(self):
        #
        f_r, f_theta = self.f_r, self.f_theta

        # v
        if f_r < 0:
            v = 0
        else:
            v = 1 * self.p.v_max * ((f_r / self.p.fix_f)**2) + self.p.v_min_2
        # w
        w = 3 * self.p.w_max * f_theta / self.p.fix_f
        if f_r < -1 and abs(w) < 0.05:
            w = 1*np.sign(w)

        # v
        if (v <= self.p.v_min_2*2) and abs(w) < 0.03:
            v = self.p.v_min_2*2

        # check bounds
        v = min(v, self.p.v_max)
        v = max(v, self.p.v_min)
        wa = min(abs(w), self.p.w_max)
        w = wa * np.sign(w)
        self.v = v
        self.w = w
