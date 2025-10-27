""" Parameters for MRAPF """

# import numpy as np


class Params:
    def __init__(self, point: bool = False):

        # configs
        self.point: bool = point
        self.simD: str = "2D"  # 3D 2D
        self.nr: int = 8
        self.method: int = 4
        self.map_id: int = 1
        self.rid: int = -1
        self.do_viz: bool = True

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
        self.local_frame = "/base_footprint"
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
        self.w_min = -1.0
        self.w_coeff = 1.0
        self.w_stall = 2.0 * 3.14/180  # rad/s

        #
        self.v_zero_tresh = 0.03
        self.w_zero_tresh = 0.03

        # thresholds
        self.goal_dis_tresh = 0.08

        # forces
        self.zeta = 1
        self.fix_f = 4
        self.fix_f2 = 10

        # radiuses and precaution distances
        self.obst_r = 0.17
        self.d_prec = 0.1
        self.robot_r = 0.2  # robots effective radius
        # #
        # self.d_prec = self.robot_r + self.obst_r + self.d_prec  # 0.57
        # self.d_start = 2 * self.d_prec
        # self.d_half = 1.5 * self.d_prec
        # self.obst_z = 4 * self.fix_f * self.d_prec**4
        #
        self.robot_d_prec = 2 * self.robot_r + self.d_prec  # 0.64
        self.robot_d_start = 2 * self.robot_d_prec
        self.robot_d_half = 1.5 * self.robot_d_prec
        self.robot_z = 4 * self.fix_f * self.robot_d_prec**4

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
