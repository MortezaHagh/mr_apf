#! /usr/bin/env python3

import rospy
import actionlib
import numpy as np
from matplotlib.pylab import plt
from plot_model import plot_model
from model_inputs import ModelInputs
from create_model import CreateModel
from Init_robot_action import InitRobotAcion
from apf.msg import InitRobotAction, InitRobotGoal


class APF(object):
    def __init__(self):

        # ros
        self.rate = rospy.Rate(5)
        rospy.on_shutdown(self.shutdown_hook)

        # model
        self.model = CreateModel(map_id=4)

        # setting
        obs_r = 1
        dt = 0.05
        zeta = 1
        d_rt = 1000
        velocities = {"v": 0.5}
        settings = {"obs_r": obs_r, "dt": dt, "zeta": zeta, "d_rt": d_rt}

        # # velocity
        # self.v = 0.5
        # self.v_max = 0.5
        # self.v_min = 0
        # self.w_max = 0.3
        # self.w_min = -0.3

        # running action servers
        common_ac_name = "/robot_action"
        self.action_names = []
        action_servers = []
        for i in range(self.model.robot_count):
            action_name = "/r" + str(self.model.robots[i].id)+common_ac_name
            ac_server = InitRobotAcion(
                self.model, i, action_name, settings, velocities)
            self.action_names.append(action_name)
            action_servers.append(ac_server)

        # calling action servers
        clients = []
        for i in range(self.model.robot_count):
            client = actionlib.SimpleActionClient(
                self.action_names[i], InitRobotAction)
            client.wait_for_server()
            goal = InitRobotGoal()
            client.send_goal(goal)
            clients.append(client)

        # wait for results
        status = [c.get_state() > 1 for c in clients]
        while 0 in status:
            status = [c.get_state() > 1 for c in clients]
            self.rate.sleep()

        self.results = []
        for c in clients:
            self.results.append(c.get_result())

        # plot
        self.plotting()

    # -----------------------  plotting  --------------------------------#

    def plotting(self):
        fig, ax = plot_model(self.model)

        # paths
        colors = plt.cm.get_cmap('rainbow', self.model.robot_count)
        for i, res in enumerate(self.results):
            ax.plot(res.path_x, res.path_y, color=colors(i))
        plt.show()

    def shutdown_hook(self):
        print(" --- shutting down from main ---")


if __name__ == "__main__":
    rospy.init_node("main_node")
    apf = APF()
    print(".........")
