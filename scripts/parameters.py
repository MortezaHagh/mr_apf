""" Parameters for MRAPF """

# import numpy as np


class Params:
    def __init__(self, point: bool = False):

        # configs
        self.point: bool = point
        self.simD: str = "3D"  # 3D 2D
        self.nr: int = 5
        self.method: int = 2
        self.map_id: int = 1
        self.rid: int = -1

        # results_folder
        self.results_folder: str = ''
        if self.point:
            self.results_folder = "results/static_tests"
            self.simD = "2D"
        else:
            self.results_folder = "results/tests"

        # topics and services names
        self.ns = ''
        self.sns = ""
        self.cmd_topic = '/cmd_vel'
        self.ac_name = "/apf_action"
        self.sru_srv_name = "/send_robot_update"
        self.global_frame = "map"
        self.odom_frame = "/odom"
        self.local_frame = "/base_footprint"  # odom
        self.fleet_data_topic = "fleet_data"

        # general
        self.path_unit = 0.7
        self.priority = self.rid
        self.dt = 0.1

        # velocities
        self.v_max = 0.2
        self.v_min = 0.0
        self.v_min_2 = 0.05  # 0.02
        self.w_max = 1.0
        self.w_min = 0.0
        self.w_coeff = 1
        #
        self.v_zero_tresh = 0.03
        self.w_zero_tresh = 0.03

        # thresholds
        self.goal_dis_tresh = 0.06

        # forces
        self.zeta = 1
        self.fix_f = 4
        self.fix_f2 = 10

        # radiuses and precaution distances
        # self.obst_r = 0.11
        self.prec_d = 0.1
        self.robot_r = 0.22  # @ 0.22        # robots effective radius
        # #
        # self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        # self.obst_start_d = 2 * self.obst_prec_d
        # self.obst_half_d = 1.5 * self.obst_prec_d
        # self.obst_z = 4 * self.fix_f * self.obst_prec_d**4
        #
        self.robot_prec_d = 2 * self.robot_r + self.prec_d  # 0.64
        self.robot_start_d = 2 * self.robot_prec_d
        self.robot_half_d = 1.5 * self.robot_prec_d
        self.robot_z = 4 * self.fix_f * self.robot_prec_d**4

        # 2d
        self.obs_effect_r = 1.0          # obstacles effective radius

    def set_ns(self, ns: str):
        self.ns = "/" + ns
        self.sns = ns
        self.ac_name = ns+self.ac_name
        self.cmd_topic = ns+self.cmd_topic

    def set_rid(self, rid: int):
        self.rid = rid
        self.priority = rid
