import numpy as np

class Params(object):
    def __init__(self, id=1):
        
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
        self.dis_tresh = 0.05
        self.ang_tresh0 = 0.02
        self.ang_tresh1 = 0.2
        self.lis_topic = '/odom'
        self.cmd_topic = '/cmd_vel'
        self.linear_max_speed = 0.2
        self.linear_min_speed = 0.01
        self.angular_max_speed = 1.0
        self.angular_min_speed = 0.0

        self.dt = 0.1
        self.zeta = 1
        self.robot_r = 1.0               # robots effective radius
        self.danger_r = 0.25             # real obst radius
        self.obs_effect_r = 1.0          # obstacles effective radius
        self.goal_distance = 1000
        self.ac_name = "/apf_action"
        self.pose_srv_name = "/pose_service"

        self.f_r_min = 0
        self.f_r_max = 5
        self.w_coeff = 1
        self.f_theta_min = 1
        self.f_theta_max = 5
        self.dis_tresh = 0.2
        self.theta_thresh = np.pi/2

        self.priority = id
