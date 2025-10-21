#! /usr/bin/env python

""" MRAPF Simulation for real-time applications """

import json
from typing import List
from matplotlib.pylab import plt
import rospy
from plotter import Plotter
from parameters import Params
from spawn_map import spawning
from create_model import MRSModel
from generate_results import Results
from visualization import RvizViusalizer
from robot_planner_l0_ac import RobotPlannerAc
from planning_clinets import PlanningClients
from central_mrapf_service import CentralMRAPF
from mrapf_classes import TestInfo, AllPlannersData
from initiate_planners import initiate_robots_planners


class Run():
    """Run the MRAPF simulation.
    """

    def __init__(self, params_i: Params = None):

        rospy.loginfo(f"[{self.__class__.__name__}]: Start running MRAPF ...")

        # settings
        params = params_i
        if params is None:
            params = Params(point=False)
        self.params: Params = params
        self.test_info: TestInfo = TestInfo(self.params)
        self.central_mrapf_srv_name = "central_mrapf_srv"

        # all simulation data, to be collected
        self.planners_data = AllPlannersData()

        # ros settings
        self.rate = rospy.Rate(20)
        self.rate40 = rospy.Rate(40)
        rospy.on_shutdown(self.shutdown_hook)

        #
        self.model: MRSModel = None
        self.visualizer: RvizViusalizer = None
        self.cmrapf: CentralMRAPF = None
        self.planning_clients: PlanningClients = None

    def run(self):

        # create model
        self.model = MRSModel(params=self.params)

        if self.params.simD == "3D":
            # spawn robots and obstacles
            spawning(model=self.model)

        # visualize
        self.visualizer = RvizViusalizer(model=self.model)

        # running central MRAPF Service Server *************************************************************************
        self.cmrapf = CentralMRAPF(self.model, self.central_mrapf_srv_name)

        # creating distributed planners - by calling central service ***************************************************
        initiate_robots_planners(self.model)
        self.rate.sleep()

        # update fleet data
        for _ in range(20):
            self.cmrapf.fleet_data_handler.update_fleet_data()
            self.rate.sleep()

        # planning clients - sending goals *****************************************************************************
        self.planning_clients = PlanningClients(self.model.robots)
        self.planning_clients.send_goals()

        # status checking
        self.check_status()

        # final results
        self.final_results_plot()

    def check_status(self):
        self.cmrapf.fleet_data_handler.update_fleet_data()
        status = [c.get_state() for c in self.planning_clients.clients]
        s_flags = [s < 2 for s in status]
        while (not rospy.is_shutdown()) and (any(s_flags)):
            self.cmrapf.fleet_data_handler.update_fleet_data()
            self.cmrapf.fleet_data_handler.check_all_stuck()
            status = [c.get_state() for c in self.planning_clients.clients]
            s_flags = [s < 2 for s in status]
            self.visualizer.update_robot_visuals(self.cmrapf.fleet_data_handler.xy)
            self.rate.sleep()

        rospy.loginfo(" ---------------------------------")
        rospy.loginfo(f"[{self.__class__.__name__}]: APF Mission Accomplished.")
        rospy.loginfo(" ---------------------------------")

    def stop_all_planners(self):
        # self.planning_clients.stop_planners()
        for ac in self.cmrapf.planners:
            ac.stop_planner()

    def final_results_plot(self):
        # planners_data
        planners: List[RobotPlannerAc] = self.cmrapf.planners
        for ac in planners:
            self.planners_data.add_data(ac.planner_data)

        # generate results
        Results(planners_data=self.planners_data, result_path=self.test_info.res_file_path, params=self.params)
        self.save_data()
        self.plotting()

    def plotting(self):
        plotter = Plotter(self.model, self.params, self.test_info.res_file_path)
        plotter.plot_all_paths(self.planners_data)
        plt.show()

    def save_data(self):
        with open(self.test_info.res_file_path + "paths.json", "w") as outfile:
            json.dump(self.planners_data.all_xy, outfile)
        # with open(self.test_info.res_file_path + "times.json", "w") as outfile:
        #     json.dump(self.planners_data.all_durations, outfile)

    def shutdown_hook(self):
        rospy.loginfo("shutting down from main ... ")
        self.stop_all_planners()


if __name__ == "__main__":
    rospy.init_node("mrapf_node")
    run = Run()
    run.run()
