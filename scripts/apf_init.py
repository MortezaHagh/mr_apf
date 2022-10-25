#! /usr/bin/env python

import json
import rospy
from parameters import Params
from matplotlib.pylab import plt
from plot_model import plot_model
from create_model import CreateModel
from apf_central_service import InitRobotService

class Run():
    def __init__(self):
        # ros
        rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # model
        self.model = CreateModel(map_id=4)
        self.count = self.model.robot_count
        self.paths = {}

        # # init_robot service server
        print("Initializing Central Service Server (init_apf_srv) for adding robots ... ")
        init_srv_name = "init_apf_srv"
        self.rsrv = InitRobotService(self.model, init_srv_name)

        while not rospy.is_shutdown():
            rate.sleep()
    
# -----------------------plotting - shutdown_hook---------------------------#

    def plotting(self):
        # map
        params = Params()
        fig, ax = plot_model(self.model, params)

        # paths
        for k, v in self.paths.items():
            ax.plot(v[0], v[1])


    def data(self):
        with open("/home/piotr/mori_ws/paths.json", "w") as outfile:
            json.dump(self.paths, outfile)


    def shutdown_hook(self):
        # paths
        for i, ac in enumerate(self.rsrv.ac_services):
            self.paths[i] = [ac.result.path_x, ac.result.path_y]

        self.data()
        self.plotting()
        plt.show()
        
        print("------------------------------------")
        print(" ----- shutting down from main -----")

# --------------------------------- __main__ ----------------------------

if __name__ == "__main__":
    
    # ros
    rospy.init_node("main_node")

    # run
    run = Run()