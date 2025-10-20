#! /usr/bin/env python

""" MRAPF Planner Base, calculate forces and velocities """

from typing import Tuple, List
import numpy as np
from geometry_msgs.msg import Pose2D
from logger import MyLogger
from parameters import Params
from mrapf_classes import ApfObstacle
from create_model import MRSModel, Robot
from apf.msg import RobotData, FleetData


class APFPlannerBase:
    """ APF Planner Base class """

    def __init__(self, model: MRSModel, robot: Robot, params: Params):
        self.lg = MyLogger(f"APFPlannerBase_r{robot.rid}")

        # data
        self.params: Params = params
        self.model: MRSModel = model
        self.robot: Robot = robot

        # obstacles
        self.obstacles: List[ApfObstacle] = []
        self.parse_map_data()

        # robot fleet data
        self.pose: Pose2D = Pose2D()
        self.robot_data: RobotData = None
        self.fleet_data: FleetData = None

        # velocity
        self.v: float = 0
        self.w: float = 0

        # forces and phi
        self.f_r: float = 0
        self.f_theta: float = 0
        self.phi: float = 0

        # forces
        self.robot_f: Tuple[float, float] = (0.0, 0.0)
        self.target_f: Tuple[float, float] = (0.0, 0.0)
        self.obs_f: Tuple[float, float] = (0.0, 0.0)

        # control vars
        self.reached = False
        self.stopped = False
        self.prev_stopped = False
        self.moving = True
        self.prev_moving = True

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
        self.moving = True

    def planner_next_move(self, pose: Pose2D, fleet_data: FleetData) -> None:
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
                self.robot_data = crob
                break

        # check dist to goal
        if self.goal_dist < self.params.goal_dis_tresh:
            self.lg.info1("reached goal!")
            self.reached = True
            return

        # calculate forces =====================================================
        if self.calculate_planner_forces():
            # calculate velocities
            self.calculate_velocity()

        # check moving
        self.eval_move_status()

    def calculate_planner_forces(self) -> bool:
        """ planner specific force calculations

        Returns:
            bool: True if robot can move, False if stopped
        """

        self.calculate_forces()
        if self.stopped:
            self.lg.warn("Robot stopped.")
            self.v = 0
            # self.w = 0
            return False
        return True

    def calculate_forces(self):
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

    def parse_map_data(self):
        # robot target
        self.goal_x = self.robot.xt
        self.goal_y = self.robot.yt
        # obstacles
        self.n_obsts = self.model.n_obstacles
        for obst in self.model.obstacles:
            apf_obst = ApfObstacle(obst, self.params)
            self.obstacles.append(apf_obst)

    def calculate_velocity(self):
        #
        f_r, f_theta = self.f_r, self.f_theta

        # v
        if f_r < 0:
            v = 0
        else:
            v = 1 * self.params.v_max * ((f_r / self.params.fix_f)**2) + self.params.v_min_2
        # w
        w = 5 * self.params.w_max * f_theta / self.params.fix_f
        if v == 0 and abs(w) < 0.05:
            w = 1*np.sign(w)

        # v
        if (v == 0) and abs(w) < 0.03:
            v = self.params.v_min_2*1

        # check bounds
        v = min(v, self.params.v_max)
        v = max(v, self.params.v_min)
        wa = min(abs(w), self.params.w_max)
        w = wa * np.sign(w)
        self.v = v
        self.w = w

    def eval_move_status(self):
        # check moving
        if abs(self.v) < self.params.v_zero_tresh and abs(self.w) < self.params.w_zero_tresh:
            self.moving = False
