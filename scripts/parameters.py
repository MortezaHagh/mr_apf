import numpy as np

class Params(object):
    def __init__(self, id = 0):
        
        self.id = id
        self.sim_params()

    def set_name_space(self, name_space):
        self.name_space = name_space
        self.ac_name = name_space+self.ac_name
        self.cmd_topic = name_space+self.cmd_topic
        self.lis_topic = name_space+self.lis_topic

    # parameters for simulation
    def sim_params(self):
        self.name_space = ''
        self.lis_topic = '/odom'
        self.cmd_topic = '/cmd_vel'
        self.ac_name = "/apf_action"
        self.pose_srv_name = "/pose_service"

        self.linear_max_speed = 0.2
        self.linear_min_speed = 0.0
        self.angular_max_speed = 1.0
        self.angular_min_speed = 0.0
        self.linear_min_speed_2 = 0.02
        
        self.dis_tresh = 0.05
        self.ang_tresh0 = 0.02
        self.ang_tresh1 = 0.2

        self.zeta = 1
        self.fix_f = 4
        self.fix_f2 = 10
        self.obst_r = 0.11
        self.prec_d = 0.07
        self.robot_r = 0.22

        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_start_d = self.obst_prec_d*2

        self.w_coeff = 1
        self.dis_tresh = 0.05 #0.2
        self.theta_thresh = 90*np.pi/180

        self.path_unit = 1.0