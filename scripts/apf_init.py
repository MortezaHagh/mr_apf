#! /usr/bin/env python

import json
import rospy
import rospkg
from parameters import Params
from matplotlib.pylab import plt
from plotter import plot_model
from create_model import MRSModel
from apf_central_service import InitRobotService

class Run():
    def __init__(self):

        # # results
        self.test = "T1"
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        self.pred = pkg_path + "/results/"
        self.dir_p = self.pred + self.test + "/apf_paths.json"
        self.dir_t = self.pred + self.test + "/apf_times.json"
        self.dir_f = self.pred + self.test + "/apf_paths"

        # ros
        rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # model
        self.model = MRSModel(map_id=4)
        self.count = self.model.n_robots
        self.paths = {}
        self.times = {}

        # # init_robot service server
        print("Initializing Central Service Server (init_apf_srv) for adding robots ... ")
        init_srv_name = "init_apf_srv"
        self.rsrv = InitRobotService(self.model, init_srv_name)

        while not rospy.is_shutdown():
            rate.sleep()
    
    def plotting(self):
        # map
        params = Params()
        fig, ax = plot_model(self.model, params)

        # paths
        colors = plt.cm.get_cmap('rainbow', self.model.n_robots)
        for k, v in self.paths.items():
            ax.plot(v[0], v[1], color=colors(k))
        plt.savefig(self.dir_f, format="png")
        plt.show()


    def data(self):
        with open(self.dir_p, "w") as outfile:
            json.dump(self.paths, outfile)
        with open(self.dir_t, "w") as outfile:
            json.dump(self.times, outfile)



    def shutdown_hook(self):
        # paths and times
        for i, ac in enumerate(self.rsrv.ac_services):
            self.paths[i] = [ac.result.path_x, ac.result.path_y]
            self.times[i] = ac.time

        self.data()
        self.plotting()
        
        print("------------------------------------")
        print(" ----- shutting down from main -----")

# --------------------------------- __main__ ----------------------------

if __name__ == "__main__":
    
    # ros
    rospy.init_node("main_node")

    # run
    run = Run()