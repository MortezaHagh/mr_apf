#! /usr/bin/env python

import json
from typing import List
from matplotlib.pylab import plt
import rospy
from plotter import Plotter
from results import Results
from parameters import Params
from create_model import MRSModel, Robot
from robot_planner_ac import RobotPlannerAc
from planning_clinets import PlanningClients
from mrapf_classes import TestInfo, AllPlannersData

from fleet_data import FleetDataH
from apf_planner_2 import APFPlanner


class Run():
    rate: rospy.Rate
    rate40: rospy.Rate
    params: Params
    model: MRSModel
    test_info: TestInfo
    planners_data: AllPlannersData

    def __init__(self):

        # data
        params = Params()
        self.params = params
        self.test_info = TestInfo(params)
        self.planners_data = AllPlannersData()  # planners data (trajectory and time)

        # ros settings
        self.rate = rospy.Rate(20)
        self.rate40 = rospy.Rate(40)
        rospy.on_shutdown(self.shutdown_hook)

        # create model
        path_unit = 0.7
        model = MRSModel(map_id=1, path_unit=path_unit, n_robots=self.params.nr)
        self.model = model

        # fleet data
        fleet_data_h = FleetDataH(params)

        # planners
        r: Robot
        planners = List[APFPlanner]
        for r in self.model.robots:
            # update fleet data handler
            fleet_data_h.add_robot(r.rid, r.sns)
            fleet_data_h.update_goal(r.rid, r.xt, r.yt)
            ap = APFPlanner(model, r, params)
            planners.append(ap)

        # central service
        # update fleet data
        for _ in range(20):
            self.cmrapf.fleet_data_h.update_all()
            self.rate.sleep()

        # planning clients - sending goals *************************************
        self.planning_clients = PlanningClients(self.model.robots_data)
        self.planning_clients.send_goals()

        # status checking
        self.check_status()

        # final results
        self.final_results_plot()

    def check_status(self):
        self.cmrapf.fleet_data_h.update_all()
        status = [c.get_state() for c in self.planning_clients.clients]
        s_flags = [s < 2 for s in status]
        while (not rospy.is_shutdown()) and (any(s_flags)):
            self.cmrapf.fleet_data_h.update_all()
            status = [c.get_state() for c in self.planning_clients.clients]
            s_flags = [s < 2 for s in status]
            self.visualizer.create_robot_circles(self.cmrapf.fleet_data_h.xy)
            self.rate40.sleep()

        rospy.loginfo(" ---------------------------------")
        rospy.loginfo(f"[{self.__class__.__name__}]: APF Mission Accomplished.")
        rospy.loginfo(" ---------------------------------")

    def final_results_plot(self):
        # planners_data
        planners: List[RobotPlannerAc] = self.cmrapf.planners
        for ac in planners:
            self.planners_data.add_data(ac.planner_data)

        #
        Results(self.planners_data, self.test_info.res_file_p)
        self.save_data()
        self.plotting()

    def plotting(self):
        plotter = Plotter(self.model, self.params, self.test_info.res_file_p)
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
    rospy.init_node("mrapf_node")
    run = Run()
    rospy.spin()
