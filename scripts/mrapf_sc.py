#! /usr/bin/env python

""" MRAPF Simulation in 2D """

import json
from math import cos, sin
from typing import List, Dict
from matplotlib.pylab import plt
import rospy
from plotter import Plotter
from parameters import Params
from generate_results import Results
from create_model import MRSModel, Robot
from fleet_data_handler import FleetDataHandler
from apf_planner_base import APFPlannerBase
from apf_planner_1 import APFPlanner as APFPlanner1
from apf_planner_2 import APFPlanner as APFPlanner2
from mrapf_classes import TestInfo, AllPlannersData, PlannerData


class Run():

    def __init__(self):

        rospy.loginfo(f"[{self.__class__.__name__}]: Start running MRAPF ...")

        # data
        params = Params(static=True)
        self.params = params
        self.test_info = TestInfo(params)

        # ros settings
        self.rate = rospy.Rate(20)
        self.rate40 = rospy.Rate(40)
        rospy.on_shutdown(self.shutdown_hook)

        #
        self.model: MRSModel = None
        self.fdh: FleetDataHandler = None
        self.planners: List[APFPlannerBase] = []
        self.apd: Dict[int, PlannerData]  # all planners data
        self.planners_data: AllPlannersData

    def run(self):

        # create model
        path_unit = 0.7
        model = MRSModel(map_id=self.params.map_id, path_unit=path_unit, n_robots=self.params.nr)
        self.model = model

        # initialize planners data
        self.apd = {}
        self.planners_data = AllPlannersData()
        for r in model.robots:
            self.apd[r.rid] = PlannerData()
            self.apd[r.rid].set_start_time(rospy.get_time())

        # fleet data
        fleet_data_h = FleetDataHandler(self.params)

       # planners - set fleet_data_h
        r: Robot
        planners: List[APFPlannerBase] = []
        for r in self.model.robots:
            # update fleet data handler
            fleet_data_h.add_robot(r.rid, r.sns)
            fleet_data_h.update_goal(r.rid, r.xt, r.yt)
            fleet_data_h.set_robot_pose(r.rid, r.xs, r.ys, r.heading)
            fleet_data_h.set_robot_data(r.rid, False, False)
            planners.append(self.create_planner(r))
        self.planners = planners
        self.fdh = fleet_data_h

        # start planning
        f = 20
        self.dt = 1/f
        self.planning()

        # final results
        self.final_results_plot()

        rospy.loginfo(" ---------------------------------")
        rospy.loginfo(f"[{self.__class__.__name__}]: APF Mission Accomplished.")
        rospy.loginfo(" ---------------------------------")

    def planning(self):
        # initialize reaches
        reaches: Dict[int, bool] = {}
        for r in self.model.robots:
            reaches[r.rid] = False

        # planning and moving
        pl: APFPlannerBase
        while not all(reaches.values()):
            for pl in self.planners:
                if pl.reached:
                    continue

                # pland and move
                pose = self.fdh.get_robot_pose(pl.robot.rid)
                fleet_data = self.fdh.get_fleet_data()
                pl.planner_move(pose, fleet_data)

                # update fleet data
                self.apd[pl.robot.rid].add_xy(pose.x, pose.y)
                self.apd[pl.robot.rid].add_vw(pl.v, pl.w)
                self.apd[pl.robot.rid].steps += 1

                # check stop
                if pl.stopped != pl.prev_stopped:
                    self.fdh.set_robot_data(pl.robot.rid, pl.stopped, False)
                    pl.prev_stopped = pl.stopped

                # check reach
                if pl.reached:
                    reaches[pl.robot.rid] = True
                    self.fdh.set_robot_data(pl.robot.rid, True, True)
                    self.apd[pl.robot.rid].set_end_time(rospy.get_time())
                    self.apd[pl.robot.rid].finalize()

                # sim move
                pose.x += (pl.v * self.dt) * cos(pose.theta)
                pose.y += (pl.v * self.dt) * sin(pose.theta)
                pose.theta += pl.w * self.dt
                pl.pose = pose
                self.fdh.set_robot_pose(pl.robot.rid, pose.x, pose.y, pose.theta)

                # log motion
                self.log_motion(pl)

    def create_planner(self, robot: Robot) -> APFPlannerBase:
        if self.params.method == 1:
            planner = APFPlanner1(self.model, robot, self.params)
        elif self.params.method == 2:
            planner = APFPlanner2(self.model, robot, self.params)
        else:
            raise ValueError("method not defined")
        return planner

    def log_motion(self, pl: APFPlannerBase):
        n = 1
        rid = pl.robot.rid
        if rid == n:
            v = round(pl.v, 2)
            w = round(pl.w, 2)
            fr = round(pl.f_r, 2)
            ft = round(pl.f_theta, 2)
            # rospy.loginfo(f"[planner_base, {self.ns}]: {self.ap.stop_flag_multi}")
            rospy.loginfo(f"[planner_base, r{rid}]: f_r: {fr}, f_theta: {ft}")
            rospy.loginfo(f"[planner_base, r{rid}]: v: {v}, w: {w}")
            rospy.loginfo(" --------------------------------------------------- ")

    def final_results_plot(self):
        # planners_data
        for pd in self.apd.values():
            self.planners_data.add_data(pd)

        # generate results
        Results(self.planners_data, self.test_info.res_file_path)
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
        pass


if __name__ == "__main__":
    rospy.init_node("mrapf_node")
    run = Run()
    run.run()
