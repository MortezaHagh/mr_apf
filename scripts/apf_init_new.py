""" MRPP APF 3D sim """
#! /usr/bin/env python

import os
import json
import rospy
import rospkg
from results import Results
from parameters import Params
from spawn_map import Spawning
from matplotlib.pylab import plt
from send_goals import SendGoal
from plotter import plot_model
from create_model import MRSModel
from visualization import Viusalize
from call_apf_service import call_apf_service
from apf_central_service import InitRobotService


class Run():
    def __init__(self):

        # test name and version
        version = 0
        self.test_id = 2

        # initialize
        self.initialize(version)

        # create model
        path_unit = 0.7
        n_robots = self.test_id
        self.model = MRSModel(map_id=1, path_unit=path_unit, n_robots=n_robots)

        # set path unit
        self.path_unit = 1.0

        # spawn robots and obstacles
        Spawning(self.model, self.path_unit)

        # visualize
        self.visualize = Viusalize(self.model)

        # init_robot service server
        print("Initializing Central Service Server (init_apf_srv) for adding robots ... ")
        init_srv_name = "init_apf_srv"
        self.rsrv = InitRobotService(self.model, init_srv_name)

        # calling services
        call_apf_service(self.model.robots_i.ids)
        self.rate.sleep()

        # send goals
        self.clients = SendGoal(self.model.robots_i)

        # status checking
        self.check_status()

        # final results
        self.final_results_plot()

    def initialize(self, version):
        self.test_name = "T" + str(self.test_id) + "_V" + str(version)
        result_name = "res_" + str(self.test_id) + "_v" + str(version) + ".json"

        # # results path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        pred = os.path.join(pkg_path, "Results-APF/Tests")
        self.pred = os.path.join(pred, self.test_name)
        # self.pred = "/home/morteza/Documents/Morteza/CurrentAPF/"
        self.dir_f = os.path.join(self.pred, "apf_paths")
        self.dir_p = os.path.join(self.pred, "apf_paths.json")
        self.dir_t = os.path.join(self.pred, "apf_times.json")
        self.dir_force = os.path.join(self.pred, "apf_forces")
        self.result_path = os.path.join(self.pred, result_name)

        # Create directory
        path = self.pred
        isExist = os.path.exists(path)
        if not isExist:
            os.makedirs(path)

        # ros settings
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # results preallocation (trajectory and time)
        self.paths = {}
        self.times = {}

    def check_status(self):
        # status checking
        status = [c.get_state() for c in self.clients.clients]
        s_flags = [s < 2 for s in status]
        while (not rospy.is_shutdown()) and (any(s_flags)):
            status = [c.get_state() for c in self.clients.clients]
            s_flags = [s < 2 for s in status]
            self.visualize.robots_circles(self.rsrv.pose_srv.xy)
            self.rate.sleep()

        print(" -----------------------")
        print("APF Mission Accomplished.")
        print(" -----------------------")

    def final_results_plot(self):
        # paths and times
        for i, ac in enumerate(self.rsrv.ac_services):
            self.paths[i] = [ac.result.path_x, ac.result.path_y]
            self.times[i] = ac.time

        Results(self.paths, self.times, self.path_unit, self.test_name, self.result_path)
        self.data()
        self.plotting()

    def plotting(self):
        # map
        params = Params()
        fig, ax = plot_model(self.model, params)

        # traversed trajectory
        colors = plt.cm.get_cmap('rainbow', len(self.paths))
        for k, v in self.paths.items():
            ax.plot(v[0], v[1], color=colors(k))
        plt.savefig(self.dir_f + ".svg", format="svg", dpi=1000)
        plt.savefig(self.dir_f + ".png", format="png", dpi=1000)

        # # forces
        # plot_forces(self.rsrv.ac_services[0], self.dir_force)

        plt.show()

    # save traversed trajectory and times
    def data(self):
        with open(self.dir_p, "w") as outfile:
            json.dump(self.paths, outfile)
        with open(self.dir_t, "w") as outfile:
            json.dump(self.times, outfile)

    def shutdown_hook(self):
        pass
        # print(" ----- shutting down from main -----")


if __name__ == "__main__":

    # ros
    rospy.init_node("main_node")

    # run
    run = Run()
