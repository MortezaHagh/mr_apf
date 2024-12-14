#! /usr/bin/env python

import json
import rospy
from typing import List
from results import Results
from parameters import Params
from matplotlib.pylab import plt
from create_model import MRSModel
from plotter import Plotter
from spawn_map import Spawning
from visualization import RvizViusalizer
from planning_clinets import PlanningClients
from mrapf_data import TestInfo, AllPlannersData
from central_mrapf_service import CentralMRAPF
from robot_planner_server import RobotPlanner
from initiate_planners import initiate_robots_planners


class Run():
    test_info: TestInfo
    rate: rospy.Rate
    planners_data: AllPlannersData
    model: MRSModel

    def __init__(self):

        # data
        self.test_info = TestInfo(v=1, n_robots=2)  # test info
        self.planners_data = AllPlannersData()  # planners data (trajectory and time)

        # ros settings
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # create model
        path_unit = 0.7
        self.model = MRSModel(map_id=1, path_unit=path_unit, n_robots=self.test_info.n_robots)

        # spawn robots and obstacles
        Spawning(model=self.model, path_unit=1.0)

        # visualize
        self.visualizer = RvizViusalizer(model=self.model)

        # init_robot service server
        print("Initializing Central Service Server (central_mrapf_srv) for adding robots ... ")
        central_mrapf_srv_name = "central_mrapf_srv"
        self.cmrapf = CentralMRAPF(self.model, central_mrapf_srv_name)

        # calling services
        initiate_robots_planners(self.model.robots_data.ids)
        self.rate.sleep()

        # planning clients - send goals
        self.planning_clients = PlanningClients(self.model.robots_data)
        self.planning_clients.send_goals()

        # status checking
        self.check_status()

        # final results
        self.final_results_plot()

    def check_status(self):
        status = [c.get_state() for c in self.planning_clients.clients]
        s_flags = [s < 2 for s in status]
        while (not rospy.is_shutdown()) and (any(s_flags)):
            status = [c.get_state() for c in self.planning_clients.clients]
            s_flags = [s < 2 for s in status]
            self.visualizer.create_robot_circles(self.cmrapf.pose_srv.xy)
            self.rate.sleep()

        print(" -----------------------")
        print("APF Mission Accomplished.")
        print(" -----------------------")

    def final_results_plot(self):
        # planners_data
        planners: List[RobotPlanner] = self.cmrapf.planners
        for ac in planners:
            self.planners_data.add_data(ac.p_data)

        #
        Results(self.planners_data, self.test_info.res_file_p)
        self.save_data()
        self.plotting()

    def plotting(self):
        # map
        params = Params()
        plotter = Plotter(self.model, params, self.test_info.res_file_p)
        plotter.plot_all_paths(self.planners_data)
        plt.show()

    def save_data(self):
        with open(self.test_info.res_file_p + "paths.json", "w") as outfile:
            json.dump(self.planners_data.all_xy, outfile)
        with open(self.test_info.res_file_p + "times.json", "w") as outfile:
            json.dump(self.planners_data.all_times, outfile)

    def shutdown_hook(self):
        pass


if __name__ == "__main__":
    rospy.init_node("main_node")
    run = Run()
