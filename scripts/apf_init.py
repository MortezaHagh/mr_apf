#! /usr/bin/env python

import rospy
import actionlib
from parameters import Params
from matplotlib.pylab import plt
from plot_model import plot_model
from create_model import CreateModel
from pose_service import PoseService
from robot_action_dynamic import InitRobotAcion
from apf.msg import InitRobotAction, InitRobotGoal
from apf_central_service import InitRobotService

class APF(object):
    def __init__(self):

        # preallocation
        self.ac_clients = []
        self.action_names = []
        self.action_servers = []

        # ros
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # model
        self.model = CreateModel(map_id=5)
        self.count = self.model.robot_count

        # setting - parameters
        params = []
        pose_srv_name = "/pose_service"
        common_ac_name = "/robot_action"
        for i in range(self.count):
            params.append(Params(pose_srv_name, common_ac_name, i))
            # params[-1].set_name_space("/r"+str(i))
            self.action_names.append(params[-1].action_name)
        self.params = params

        # robots poses
        robot_poses = []
        for i in range(self.count):
            pose = [self.model.robots[i].xs, self.model.robots[i].ys]
            robot_poses.append(pose)

        # pose service
        count = self.model.robot_count
        self.pose_srv_name = pose_srv_name
        self.pose_srv = PoseService(robot_poses, count, self.pose_srv_name)

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
        for i in range(self.count):
            ac_server = InitRobotAcion(self.params[i], self.model)
            self.action_servers.append(ac_server)

        # calling action servers
        for i in range(self.count):
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
        fig, ax = plot_model(self.model, self.params[0])

        # paths
        colors = plt.cm.get_cmap('rainbow', self.model.robot_count)
        for i, res in enumerate(self.results):
            ax.plot(res.path_x, res.path_y, color=colors(i))
        
        fig1, ax1 = plt.subplots(1, 1)
        ax1.plot(self.action_servers[0].force_r, label="force_r")
        ax1.plot(self.action_servers[0].force_t, label="force_t")
        ax1.set_title("forces")
        ax1.legend()

        fig2, ax2 = plt.subplots(1, 1)
        ax2.plot(self.action_servers[0].v_lin, label="v_lin")
        ax2.plot(self.action_servers[0].v_ang, label="v_ang")
        ax2.legend()
        
        plt.show()


# ------------------------------------------------------------------- #

def plotting(model):
    params = Params()
    fig, ax = plot_model(model, params)
    return fig, ax


def shutdown_hook():
    plt.show()
    print("-----------------------------------")
    print(" --- shutting down from main ---")

# --------------------------------- __main__ ----------------------------
 
if __name__ == "__main__":
    
    # ros
    rospy.init_node("main_node")
    rate = rospy.Rate(20)
    rospy.on_shutdown(shutdown_hook)

    # model
    model = CreateModel(map_id=4)
    count = model.robot_count

    # plotting
    fig, ax = plotting(model)
  
    # # init_robot service server
    # """" init_apf_srv """""
    print("Initializing Central Service Server (init_apf_srv) for adding robots ... ")
    init_srv_name = "init_apf_srv"
    rsrv = InitRobotService(model, init_srv_name, ax)


    while not rospy.is_shutdown():
        rate.sleep()

