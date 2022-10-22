
class Params(object):
    def __init__(self):
        self.sim_params()

    def set_name_space(self, name_space):
        self.name_space = name_space
        self.cmd_topic = name_space+self.cmd_topic
        self.lis_topic = name_space+self.lis_topic
        self.action_name = name_space+self.action_name

    # parameters for simulation
    def sim_params(self):
        self.lis_topic = '/odom'
        self.action_name = "/motion_action"
        self.cmd_topic = '/cmd_vel'
        self.path_strategy = ""  # summery
        self.angular_max_speed = 1.0  # 0.8
        self.linear_max_speed = 0.2  # 0.15
        self.angular_min_speed = 0.0
        self.linear_min_speed = 0.05
        self.dis_tresh = 0.05
        self.ang_tresh0 = 0.02
        self.ang_tresh1 = 0.2
        self.name_space = ''
