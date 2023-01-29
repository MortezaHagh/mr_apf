#! /usr/bin/env python

import json
import rospy
import rospkg
from results import Results
from parameters import Params
from spawn_map import Spawning
from matplotlib.pylab import plt
from  send_goals import SendGoal
from broadcaster import BroadCast
from plot_model import plot_model
from plot_forces import plot_forces
from create_model import CreateModel
from call_apf_service import call_apf_service
from apf_central_service import InitRobotService


class Run():
    def __init__(self):

        # # results
        self.test_id = 1                              # check 3 !!!!!!!!!!
        self.test = "T" + str(self.test_id)
        # rospack = rospkg.RosPack()
        # pkg_path = "rospack.get_path('apf')"
        self.pred = "/home/piotr/Documents/Morteza/CurrentAPF/"
        self.dir_p = self.pred + self.test + "/apf_paths.json"
        self.dir_t = self.pred + self.test + "/apf_times.json"
        self.dir_f = self.pred + self.test + "/apf_paths"
        self.dir_force = self.pred + self.test + "/apf_forces"
        res_pred = "res_" + str(self.test_id) + ".json"
        self.result_path = "/home/piotr/Documents/Morteza/CurrentAPF" + '/result_apf/' + res_pred

        # ros
        rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # model
        self.model = CreateModel(map_id=-1)                # check 3 !!!!!!!!!!
        self.count = self.model.robot_count
        self.paths = {}
        self.times = {}

        # spawn
        Spawning(self.model)

        # # init_robot service server ------------------------------------------------------
        print("Initializing Central Service Server (init_apf_srv) for adding robots ... ")
        init_srv_name = "init_apf_srv"
        self.rsrv = InitRobotService(self.model, init_srv_name)

        # ------------------------- call_init_service - SendGoal - status---------

        # calling services
        call_apf_service(self.model.robots_i.ids)
        rate.sleep()

        # send goals
        clients = SendGoal(self.model.robots_i)

        # status checking
        status = [c.get_state() for c in clients.clients]
        s_flags = [s<2 for s in status]
        while (not rospy.is_shutdown()) and (any(s_flags)):
            rate.sleep()
            status = [c.get_state() for c in clients.clients]
            s_flags = [s<2 for s in status]
        print(" -----------------------")
        print("APF Mission Accomplished.")
        print(" -----------------------")

        # --------------------------------- results ---------------------
        # paths and times
        for i, ac in enumerate(self.rsrv.ac_services):
            self.paths[i] = [ac.result.path_x, ac.result.path_y]
            self.times[i] = ac.time
        
        Results(self.paths, self.times, self.model.path_unit, self.test_id, self.result_path)
        self.data()
        self.plotting()

    # -----------------------plotting - shutdown_hook---------------------------#

    def plotting(self):
        # map
        params = Params()
        fig, ax = plot_model(self.model, params)

        # paths
        colors = plt.cm.get_cmap('rainbow', len(self.paths))
        for k, v in self.paths.items():
            ax.plot(v[0], v[1], color=colors(k))
        plt.savefig(self.dir_f+".png", format="png")
        
        plot_forces(self.rsrv.ac_services[0], self.dir_force)
        plt.show()


    def data(self):
        with open(self.dir_p, "w") as outfile:
            json.dump(self.paths, outfile)
        with open(self.dir_t, "w") as outfile:
            json.dump(self.times, outfile)


    def shutdown_hook(self):
        pass
        # print(" ----- shutting down from main -----")

# --------------------------------- __main__ ----------------------------

if __name__ == "__main__":
    
    # ros
    rospy.init_node("main_node")

    # run
    run = Run()