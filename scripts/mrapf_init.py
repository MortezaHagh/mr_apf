#! /usr/bin/env python

import json
from typing import List
from matplotlib.pylab import plt
import rospy
from plotter import Plotter
from results import Results
from parameters import Params
from spawn_map import Spawning
from create_model import MRSModel
from visualization import RvizViusalizer
from planning_clinets import PlanningClients
from robot_planner_server import RobotPlanner
from central_mrapf_service import CentralMRAPF
from mrapf_classes import TestInfo, AllPlannersData
from initiate_planners import initiate_robots_planners


class Run():
    rate: rospy.Rate
    model: MRSModel
    test_info: TestInfo
    planners_data: AllPlannersData

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

        # running central MRAPF Service Server *********************************
        central_mrapf_srv_name = "central_mrapf_srv"
        self.cmrapf = CentralMRAPF(self.model, central_mrapf_srv_name)

        # creating distributed planners - by calling central service ***********
        initiate_robots_planners(self.model.robots_data.ids)
        self.rate.sleep()

        # planning clients - sending goals *************************************
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

        rospy.loginfo(" ---------------------------------")
        rospy.loginfo(f"[{self.__class__.__name__}]: APF Mission Accomplished.")
        rospy.loginfo(" ---------------------------------")

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
        # with open(self.test_info.res_file_p + "times.json", "w") as outfile:
        #     json.dump(self.planners_data.all_times, outfile)

    def shutdown_hook(self):
        pass


if __name__ == "__main__":
    rospy.init_node("main_node")
    run = Run()
