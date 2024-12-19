import numpy as np


class Params:
    id: int
    ns: str
    ac_name: str
    cmd_topic: str
    lis_topic: str
    name_space: str

    def __init__(self, rid=0):
        self.id = rid
        self.ns = None
        self.name_space = None
        self.ac_name = None
        self.cmd_topic = None
        self.lis_topic = None
        self.sim_params()

    def set_name_space(self, name_space: str):
        self.ns = name_space
        self.name_space = name_space
        self.ac_name = name_space+self.ac_name
        self.cmd_topic = name_space+self.cmd_topic
        self.lis_topic = name_space+self.lis_topic

    # parameters for simulation
    def sim_params(self):
        # proccess names
        self.ns = ''
        self.name_space = ''
        self.lis_topic = '/odom'
        self.cmd_topic = '/cmd_vel'
        self.ac_name = "/apf_action"
        self.pose_srv_name = "/pose_service"

        # general
        self.path_unit = 1.0
        self.priority = id
        self.dt = 0.1

        # velocities
        self.v_max = 0.2
        self.v_min = 0.01  # @ 0.0
        self.w_max = 1.0
        self.w_min = 0.0
        self.v_min_2 = 0.02
        self.w_coeff = 1

        # thresholds
        self.dis_tresh = 0.2  # @ 0.05
        self.ang_tresh0 = 0.02
        self.ang_tresh1 = 0.2
        self.theta_thresh = 90*np.pi/180
        self.goal_dis_tresh = 0.06

        # forces
        self.zeta = 1
        self.fix_f = 4
        self.fix_f2 = 10

        # radiuses and precaution distances
        self.obst_r = 0.11
        self.prec_d = 0.07
        self.robot_r = 1.0  # @ 0.22        # robots effective radius
        #
        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_start_d = 2 * self.obst_prec_d
        self.obst_half_d = 1.5 * self.obst_prec_d
        self.obst_z = 4 * self.fix_f * self.obst_prec_d**4
        #
        self.robot_prec_d = 2 * self.robot_r + self.prec_d  # 0.64
        self.robot_start_d = 2 * self.robot_prec_d
        self.robot_half_d = 1.5 * self.robot_prec_d
        self.robot_z = 4 * self.fix_f * self.robot_prec_d**4

        # settings
        #
        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_start_d = 2 * self.obst_prec_d
        self.obst_half_d = 1.5 * self.obst_prec_d
        self.obst_z = 4 * self.fix_f * self.obst_prec_d**4
        #
        self.robot_prec_d = 2 * self.robot_r + self.prec_d  # 0.64
        self.robot_start_d = 2 * self.robot_prec_d
        self.robot_half_d = 1.5 * self.robot_prec_d
        self.robot_z = 4 * self.fix_f * self.robot_prec_d**4

        # # for static -----------

        # self.danger_r = 0.25             # real obst radius
        self.obs_effect_r = 1.0          # obstacles effective radius
        self.goal_distance = 1000

        #
        self.f_r_min = 0
        self.f_r_max = 5
        self.f_theta_min = 1
        self.f_theta_max = 5
