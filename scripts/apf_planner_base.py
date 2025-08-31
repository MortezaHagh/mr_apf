#! /usr/bin/env python

""" MRAPF Planner Base, calculate forces and velocities """

from typing import Tuple
import numpy as np
from geometry_msgs.msg import Pose2D
from parameters import Params
from create_model import MRSModel, Robot
from apf.msg import RobotData, FleetData


class APFPlannerBase:
    """ APF Planner Base class """
    p: Params
    model: MRSModel
    robot: Robot
    pose: Pose2D
    robot_data: RobotData
    fleet_data: FleetData

    def __init__(self, model: MRSModel, robot: Robot, params: Params):

        # data
        self.params = params
        self.model = model
        self.robot = robot
        self.map_data()

        # robot fleet data
        self.pose = Pose2D()
        self.robot_data = None
        self.fleet_data = None

        # velocity
        self.v = 0
        self.w = 0

        # forces and phi
        self.f_r = 0
        self.f_theta = 0
        self.phi = 0

        # forces
        self.robot_f: Tuple[float, float] = (0.0, 0.0)
        self.target_f: Tuple[float, float] = (0.0, 0.0)
        self.obs_f: Tuple[float, float] = (0.0, 0.0)

        # control vars
        self.reached = False
        self.stopped = False
        self.prev_stopped = False

        # control vars
        self.ad_rg_h = None
        self.theta_rg = 0
        self.goal_theta = 0
        self.goal_dist = 1000

    def reset_vals(self):
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
        pass

    def forces(self):
        f_r = 0
        f_theta = 0

        # target
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
        pass

    def f_obstacle(self):
        pass

    def f_robots(self):
        pass

    def map_data(self):
        # robot target
        self.goal_x = self.robot.xt
        self.goal_y = self.robot.yt
        # obstacles
        self.obs_x = self.model.obsts.x
        self.obs_y = self.model.obsts.y
        self.obs_count = self.model.obsts.count
        self.obs_ind_main = [i for i in range(self.model.obsts.count)]

    def cal_vel(self):
        #
        f_r, f_theta = self.f_r, self.f_theta

        # v
        if f_r < 0:
            v = 0
        else:
            v = 1 * self.params.v_max * ((f_r / self.params.fix_f)**2) + self.params.v_min_2
        # w
        w = 3 * self.params.w_max * f_theta / self.params.fix_f
        if f_r < -1 and abs(w) < 0.05:
            w = 1*np.sign(w)

        # v
        if (v <= self.params.v_min_2*2) and abs(w) < 0.03:
            v = self.params.v_min_2*2

        # check bounds
        v = min(v, self.params.v_max)
        v = max(v, self.params.v_min)
        wa = min(abs(w), self.params.w_max)
        w = wa * np.sign(w)
        self.v = v
        self.w = w
