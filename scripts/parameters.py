import numpy as np


class Params(object):
    def __init__(self, id=0):

        self.id = id
        self.sim_params()

    def set_name_space(self, name_space):
        self.name_space = name_space
        self.ac_name = name_space+self.ac_name
        self.cmd_topic = name_space+self.cmd_topic
        self.lis_topic = name_space+self.lis_topic

    # parameters for simulation
    def sim_params(self):
        # proccess names
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
        self.v_min = 0.0
        self.w_max = 1.0
        self.w_min = 0.0
        self.v_min_2 = 0.02
        self.w_coeff = 1

        # thresholds
        self.dis_tresh = 0.05
        self.ang_tresh0 = 0.02
        self.ang_tresh1 = 0.2
        self.theta_thresh = 90*np.pi/180

        # forces
        self.zeta = 1
        self.fix_f = 4
        self.fix_f2 = 10

        # radiuses and precaution distances
        self.obst_r = 0.11
        self.prec_d = 0.07
        self.robot_r = 0.22  # @ 0.22        # robots effective radius
        #
        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_start_d = self.obst_prec_d*2
