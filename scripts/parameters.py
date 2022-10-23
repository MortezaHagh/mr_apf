import numpy as np

class Params(object):
    def __init__(self, pose_srv_name, common_ac_name, id):
        
        self.pose_srv_name = pose_srv_name
        self.action_name = common_ac_name
        self.id = id
        self.sim_params()

    def set_name_space(self, name_space):
        self.name_space = name_space
        self.cmd_topic = name_space+self.cmd_topic
        self.lis_topic = name_space+self.lis_topic
        self.action_name = name_space+self.action_name

    # parameters for simulation
    def sim_params(self):
        self.action_name = "/motion_action"
        self.cmd_topic = '/cmd_vel'
        self.lis_topic = '/odom'
        self.linear_max_speed = 0.2
        self.linear_min_speed = 0.01
        self.angular_max_speed = 1.0
        self.angular_min_speed = 0.0
        self.dis_tresh = 0.05
        self.ang_tresh0 = 0.02
        self.ang_tresh1 = 0.2
        self.name_space = ''

        self.zeta = 1
        self.robot_r = 1.0               # robots effective radius
        self.danger_r = 0.25             # real obst radius
        self.obs_effect_r = 1.0          # obstacles effective radius
        self.goal_distance = 1000
        self.pose_srv_name = "/pose_service"

        self.f_r_min = 0
        self.f_r_max = 5
        self.w_coeff = 1
        self.dis_tresh = 0.2
        self.f_theta_min = 1
        self.f_theta_max = 5
        self.theta_thresh = np.pi/2

