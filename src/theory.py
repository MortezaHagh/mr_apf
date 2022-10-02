#! /usr/bin/env python3

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
        zeta = 1
        dt = 0.2
        obs_r = 1.2
        robot_r = 1.2
        danger_r = 0.25
        d_rt = 1000
        self.velocities = {"v": 0.5}
        self.settings = {"robot_r": robot_r, "obs_r": obs_r, "dt": dt, "zeta": zeta,
                         "d_rt": d_rt}

        # ros
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # model
        self.model = CreateModel(map_id=4)

        # robots poses
        robot_poses = []
        for i in range(self.model.robot_count):
            pose = [self.model.robots[i].xs, self.model.robots[i].ys]
            robot_poses.append(pose)


        # plot
        self.plotting()

    # ----------------------- actions ----------------------------------#

    def all_move(self):
        

    # -----------------------  manage_poses  --------------------------------#

    def manage_poses(self):
        robot_poses = []
        for myac in self.action_servers:
            pose = [myac.r_x, myac.r_y]
            robot_poses.append(pose)
        self.pose_srv.update_poses(robot_poses)

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

# ------------------------------------------------------------------- #


if __name__ == "__main__":
    rospy.init_node("main_node")
    apf = APF()
    print(".........")


# # velocity
# self.v = 0.5
# self.v_max = 0.5
# self.v_min = 0
# self.w_max = 0.3
# self.w_min = -0.3
