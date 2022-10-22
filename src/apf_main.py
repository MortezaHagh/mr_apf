#! /usr/bin/env python

import rospy
import actionlib
from matplotlib.pylab import plt
from plot_model import plot_model
from create_model import CreateModel
from pose_service import PoseService
from Init_robot_action import InitRobotAcion
from apf.msg import InitRobotAction, InitRobotGoal


class APF(object):
    def __init__(self):

        # setting
        dt = 0.1
        zeta = 1
        robot_r = 1.0               # robots effective radius
        danger_r = 0.25             # real obst radius
        obs_effect_r = 1.0          # obstacles effective radius
        goal_distance = 1000
        pose_srv_name = "/pose_service"
        self.velocities = {"v": 0.5, "v_min": 0.01, "v_max": 0.2, "w_min":0, "w_max":1.0}
        self.settings = {"robot_r": robot_r, "obs_effect_r": obs_effect_r, "dt": dt, "zeta": zeta, "goal_distance": goal_distance, "pose_srv_name": pose_srv_name, "danger_r":danger_r}

        # ros
        self.rate = rospy.Rate(100)
        rospy.on_shutdown(self.shutdown_hook)

        # model
        self.model = CreateModel(map_id=4)

        # robots poses
        robot_poses = []
        for i in range(self.model.robot_count):
            pose = [self.model.robots[i].xs, self.model.robots[i].ys]
            robot_poses.append(pose)

        # pose service
        count = self.model.robot_count
        self.pose_srv_name = pose_srv_name
        self.pose_srv = PoseService(
            robot_poses, count, self.pose_srv_name)

        # actions
        self.manage_actions()

        # status check
        status = [c.get_state() > 1 for c in self.ac_clients]
        while (0 in status) and (not rospy.is_shutdown()):
            self.manage_poses()
            status = [c.get_state() > 1 for c in self.ac_clients]
            print(status)
            self.rate.sleep()

        # results
        self.results = []
        for ac_client in self.ac_clients:
            self.results.append(ac_client.get_result())

        # plot
        self.plotting()

        rospy.signal_shutdown("ended")

    # ----------------------- actions ----------------------------------#

    def manage_actions(self):
        # running action servers
        common_ac_name = "/robot_action"
        self.action_names = []
        self.action_servers = []
        for i in range(self.model.robot_count):
            action_name = "/r" + str(self.model.robots[i].id)+common_ac_name
            ac_server = InitRobotAcion(self.model, i, action_name, self.settings, self.velocities)
            self.action_names.append(action_name)
            self.action_servers.append(ac_server)

        # calling action servers
        self.ac_clients = []
        for i in range(self.model.robot_count):
            client = actionlib.SimpleActionClient(self.action_names[i], InitRobotAction)
            client.wait_for_server()
            goal = InitRobotGoal()
            client.send_goal(goal)
            self.ac_clients.append(client)

    # -----------------------  manage_poses  --------------------------------#

    def manage_poses(self):
        robot_poses = []
        for ac_server in self.action_servers:
            pose = [ac_server.r_x, ac_server.r_y]
            robot_poses.append(pose)
        self.pose_srv.update_poses(robot_poses)

    # -----------------------  plotting  --------------------------------#

    def plotting(self):
        fig, ax = plot_model(self.model, self.settings)

        # paths
        colors = plt.cm.get_cmap('rainbow', self.model.robot_count)
        for i, res in enumerate(self.results):
            ax.plot(res.path_x, res.path_y, color=colors(i))
        
        fig1, ax1 = plt.subplots(1, 1)
        ax1.plot(self.action_servers[0].v_lin)
        ax1.plot(self.action_servers[0].v_ang)
        
        plt.show()

    def shutdown_hook(self):
        # fig1, ax1 = plt.subplots(1, 1)
        # ax1.plot(self.action_servers[0].force_r)
        # ax1.plot(self.action_servers[0].force_t)
        # plt.show()
        print("-----------------------------------")
        print(" --- shutting down from main ---")

# ------------------------------------------------------------------- #


if __name__ == "__main__":
    rospy.init_node("main_node")
    apf = APF()


# # velocity
# self.v = 0.5
# self.v_max = 0.5
# self.v_min = 0
# self.w_max = 0.3
# self.w_min = -0.3
