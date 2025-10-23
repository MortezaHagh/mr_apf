#! /usr/bin/env python

from collections import defaultdict, deque
from typing import List, Dict, Tuple
import numpy as np
from shapely.geometry import Point, shape, MultiPoint
from logger import MyLogger
from parameters import Params
from mrapf_classes import ApfRobot
from apf.msg import RobotData, FleetData
from apf_planner_base import APFPlannerBase
from my_utils import cal_distance, cal_angle_diff
from create_model import MRSModel, Robot, Obstacle


class APFPlanner(APFPlannerBase):

    def __init__(self, model: MRSModel, robot: Robot, params: Params):
        APFPlannerBase.__init__(self, model, robot, params)
        self.lg = MyLogger(f"APFPlanner3_r{robot.rid}")
        self.use_fake_cluster_obsts = True

        #
        self.apf_robots: List[ApfRobot] = []
        self.fake_obsts_localmin: List[Obstacle] = []
        self.multi_robots_vis: List[ApfRobot] = []
        self.clusters_x_y: List = []

        #
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_robot_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False
        self.stop_flag_multi = False

        # config
        self.c_r_R_cluster = 3.0
        self.c_cluster_radius = 1.0
        self.c_r_is_goal_close_cluster = 2.0

    def reset_vals_2(self):
        #
        self.apf_robots = []
        self.fake_obsts_localmin = []
        self.multi_robots_vis = []
        self.clusters_x_y = []
        #
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_robot_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False
        self.stop_flag_multi = False

    def calculate_planner_forces(self) -> bool:
        """ planner specific force calculations

        Returns:
            bool: True if robot can move, False if stopped
        """

        self.reset_vals_2()

        #  to update robot data
        self.f_target()

        # detect obstacles
        self.detect_obstacles_in_proximity()

        # detect and group
        self.create_all_fake_obstacles()

        # check stop_flag_multi
        if self.stop_flag_multi:
            self.lg.warn("stop_flag_multi activated.")
            self.stopped = True
            self.v = 0
            # self.w = 0 # todo
            return False
        else:
            # calculate forces
            self.calculate_forces()

            # check stop flags
            if self.stop_flag_obsts or self.stop_flag_robots:
                self.lg.warn("stop_flag_obsts or stop_flag_robots activated.")
                self.stopped = True
                self.v = 0
                if abs(self.w) < (np.deg2rad(2)):  # todo
                    self.v = self.params.v_min_2
                return False

            # check stop_flag_robot_full
            if self.stop_flag_robot_full:
                self.lg.warn("stop_flag_robot_full activated.")
                self.stopped = True
                self.v = 0
                # self.w = 0 # todo
                return False
        return True

    def create_all_fake_obstacles(self) -> None:
        """ create all fake obstacles from fleet data. \n
        includes local minima avoidance and clustering.
        """

        # reset
        is_goal_close = False
        apf_robots: List[ApfRobot] = []
        AD_h_rR: Dict[int, float] = {}
        clust_candid_inds: List[int] = []
        multi_robots: List[ApfRobot] = []

        # is_goal_close
        goal_dist = cal_distance(self.pose.x, self.pose.y, self.goal_x, self.goal_y)
        if goal_dist < (self.c_r_is_goal_close_cluster*self.params.robot_start_d):
            is_goal_close = True

        # evaluate fleet data ##################################################
        # create new apf_robots and
        apf_robots, clust_candid_inds, AD_h_rR = self.eval_fleet_data()
        self.apf_robots = apf_robots

        # create fake_apf_robots_1 for local minima avoidance ##################
        # fake_apf_robots_1: List[ApfRobot]
        self.create_fake_obsts_loc_min_avoid(apf_robots)

        if not self.use_fake_cluster_obsts:
            return

        # if there is none robots in proximity
        # if goal is close, don't create fake obstacles from clustering
        if not (is_goal_close or len(clust_candid_inds) == 0):

            # create fake obstacles from clustering ############################
            multi_robots = self.create_fake_obst_clusters(clust_candid_inds, AD_h_rR)
            if len(multi_robots) > 0:
                self.multi_robots_vis.append(multi_robots[0])
                self.apf_robots.append(multi_robots[0])

        return

    def detect_obstacles_in_proximity(self):
        """ detect obstacles in proximity circle"""
        f_obsts_inds = []
        for i, obst in enumerate(self.obstacles):
            do = cal_distance(obst.x, obst.y, self.pose.x, self.pose.y)
            if do < obst.obst_start_d:
                f_obsts_inds.append(i)
        self.f_obsts_inds = f_obsts_inds

    def eval_fleet_data(self) -> Tuple[List[ApfRobot], List[int], Dict[int, float]]:
        """
        evaluate fleet data and create ApfRobot instances for robots in proximity circle.

        Returns:
            tuple containing
            - apf_robots(List[ApfRobot]): list of new ApfRobot instances based on fleet data
            - clust_candid_inds(List[int]): list of robots indices for clustering
            - AD_h_rR(Dict[int, float]): angle differences of heading with rR to other robots
        """

        apf_robots: List[ApfRobot] = []
        clust_candid_inds: List[int] = []
        AD_h_rR: Dict[int, float] = {}

        # eval fleet data
        # find robots in proximity circle for clustering and create ApfRobot instances
        fd: FleetData = self.fleet_data
        orob: RobotData = None
        for orob in fd.fdata:
            if orob.rid == self.robot.rid:
                continue
            dx = orob.x - self.pose.x
            dy = orob.y - self.pose.y
            d_rR = np.sqrt(dx**2 + dy**2)

            # check distance for clustering
            if d_rR > (self.c_r_R_cluster * self.params.robot_start_d):
                continue

            # calculate angle difference
            theta_rR = np.arctan2(dy, dx)
            ad_h_rR = cal_angle_diff(self.pose.theta, theta_rR)
            AD_h_rR[orob.rid] = ad_h_rR

            # # add robot for clustering
            # check if robot is not reached, add to clust_candid_inds
            if not orob.reached:
                # other Robot is not between robot and goal
                ad_rg_rR = cal_angle_diff(self.theta_rg,  theta_rR)
                if abs(ad_rg_rR) < (np.pi/2):
                    clust_candid_inds.append(orob.rid)

            # create ApfRobot instances as individual robots
            if d_rR < (1 * self.params.robot_start_d):
                new_robot = self.create_new_apf_robot(orob, d_rR, ad_h_rR, theta_rR)
                apf_robots.append(new_robot)

        return apf_robots, clust_candid_inds, AD_h_rR

    def create_fake_obsts_loc_min_avoid(self, apf_robots: List[ApfRobot]) -> None:
        """ create fake obstacles between robot and obstacle.
            for local minima avoidance
        """
        fake_obsts: List[Obstacle] = []
        for orob in apf_robots:
            if not orob.reached:
                continue
            obst: Obstacle = None
            for obst in self.obstacles:
                xo = obst.x
                yo = obst.y
                d_Ro = cal_distance(xo, yo, orob.x, orob.y)
                if d_Ro < (orob.r_prec + obst.obst_prec_d) and d_Ro > (orob.r_prec + obst.obst_prec_d)*0.75:
                    x = (xo+orob.x)/2
                    y = (yo+orob.y)/2
                    r = d_Ro/3
                    fake_obsts.append(Obstacle(x=x, y=y, r=r, params=self.params))
        self.fake_obsts_localmin = fake_obsts

    def create_fake_obst_clusters(self, clust_candid_inds: List[int], AD_h_rR: Dict[int, float]) -> List[ApfRobot]:
        """ create fake obstacles from clustering
        """

        # initialize
        nr_multi_robots = []
        multi_robots: List[ApfRobot] = []

        # fleet data
        fd: FleetData = self.fleet_data

        # Build adjacency list for clustering
        adjacents = defaultdict(list)
        for i, ind in enumerate(clust_candid_inds):
            for ind_j in clust_candid_inds[i+1:]:
                dist = cal_distance(fd.fdata[ind].x, fd.fdata[ind].y, fd.fdata[ind_j].x, fd.fdata[ind_j].y)
                if dist < (self.params.robot_prec_d*2.2):    # param 2 todo
                    adjacents[ind].append(ind_j)
                    adjacents[ind_j].append(ind)

        # BFS to find connected components (clusters)
        clusters = []
        visited = {ind: False for ind in clust_candid_inds}
        for ind in clust_candid_inds:
            if not visited[ind]:
                cluster = []
                queue = deque([ind])
                while queue:
                    current = queue.popleft()
                    if not visited[current]:
                        visited[current] = True
                        cluster.append(current)
                        for neighbor in adjacents[current]:
                            if not visited[neighbor]:
                                queue.append(neighbor)
                clusters.append(cluster)

        # Fake obstacles from clusters
        for cluster in clusters:
            if len(cluster) <= 1:
                continue
            is_robot_in = False
            is_target_in = False

            # polygon
            if len(cluster) == 2:
                x1, y1 = fd.fdata[cluster[0]].x, fd.fdata[cluster[0]].y
                x2, y2 = fd.fdata[cluster[1]].x, fd.fdata[cluster[1]].y
                xc = (x1 + x2) / 2
                yc = (y1 + y2) / 2
                rc = cal_distance(x1, y1, x2, y2) / 2 + self.params.robot_r * self.c_cluster_radius
            else:
                robot_p = Point(self.pose.x, self.pose.y)
                target_p = Point(self.goal_x, self.goal_y)
                polys_points = [Point(fd.fdata[i].x, fd.fdata[i].y) for i in cluster]
                if len(polys_points) > 2:
                    multipoint = MultiPoint([shape(p) for p in polys_points])
                    mp_ch = multipoint.convex_hull
                    is_robot_in = mp_ch.contains(robot_p)
                    is_target_in = mp_ch.contains(target_p)
                    x_y = mp_ch.boundary.coords.xy
                    self.clusters_x_y.append(x_y)

                    # get the minimum bounding circle of the convex hull
                    poly_xys = list(zip(*x_y))
                    xc = sum(x for x, y in poly_xys) / len(poly_xys)
                    yc = sum(y for x, y in poly_xys) / len(poly_xys)
                    d_c_max = max(cal_distance(xc, yc, x, y) for x, y in poly_xys)

                    # # get the minimum bounding circle of the convex hull
                    # mbr = mp_ch.minimum_rotated_rectangle
                    # mbr_c = mbr.centroid.coords[0]
                    # xc = mbr_c[0]
                    # yc = mbr_c[1]
                    # # get distance from center to furthest point to
                    # d_c_max = max(cal_distance(mbr_c[0], mbr_c[1], x, y) for x, y in zip(*poly_xys))

                    #
                    rc = d_c_max + self.params.robot_r * self.c_cluster_radius
                    # rc = max(rc, 2*self.params.robot_prec_d)

            # if robot is in the polygon
            if is_robot_in and not is_target_in:
                self.stop_flag_multi = True
                return multi_robots

            # check target in
            d_cg = cal_distance(self.goal_x, self.goal_y, xc, yc)
            if (d_cg < rc):
                continue

            # calculate distance and angle to cluster center
            dx = xc - self.pose.x
            dy = yc - self.pose.y
            d_rR = np.sqrt(dx**2 + dy**2)
            theta_rR = np.arctan2(dy, dx)
            ad_h_rR = cal_angle_diff(self.pose.theta, theta_rR)

            # create fake robot
            fake_r = ApfRobot(xc, yc, d_rR, H=self.pose.theta-np.pi)
            fake_r.cluster = True
            fake_r.prior = False  # dec_var
            fake_r.ad_h_rR = ad_h_rR
            fake_r.theta_rR = theta_rR
            fake_r.ad_rg_rR = cal_angle_diff(self.theta_rg, theta_rR)
            fake_r.r_prec = rc + self.params.robot_r + self.params.prec_d
            fake_r.r_half = 1.5 * fake_r.r_prec
            fake_r.r_start = 2.0 * fake_r.r_prec
            fake_r.z = 4 * self.params.fix_f * fake_r.r_prec**4
            multi_robots.append(fake_r)
            nr_multi_robots.append(len(cluster))

        if len(multi_robots) > 1:
            max_ind = np.argmax(nr_multi_robots)
            multi_robots = [multi_robots[max_ind]]
        return multi_robots

    def create_new_apf_robot(self, orob: RobotData, d_rR: float, ad_h_rR: float, theta_rR: float) -> ApfRobot:
        """ create new ApfRobot instance for calculating forces
        """
        robo = ApfRobot(x=orob.x, y=orob.y, d=d_rR, H=orob.h)
        robo.ad_h_rR = ad_h_rR
        robo.theta_rR = theta_rR
        robo.priority = orob.priority
        robo.prior = (not (orob.reached or orob.stopped)) and (orob.priority > self.robot_data.priority)
        robo.stopped = orob.stopped
        robo.reached = orob.reached
        robo.ad_rg_rR = cal_angle_diff(self.theta_rg, theta_rR)
        r_prec = self.params.robot_prec_d
        robo.r_prec = r_prec
        robo.r_half = 1.5 * r_prec
        robo.r_start = 2.0 * r_prec
        robo.z = 4 * self.params.fix_f * r_prec**4
        return robo

    def f_target(self):
        # r_g
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        goal_dist = np.sqrt(dx**2 + dy**2)
        # f = self.params.zeta * goal_dist
        f = self.params.fix_f
        if goal_dist < 0.5:
            f = 2*f
        theta_rg = np.arctan2(dy, dx)
        ad_rg_h = cal_angle_diff(theta_rg, self.pose.theta)
        self.ad_rg_h = ad_rg_h
        self.theta_rg = theta_rg
        self.goal_dist = goal_dist
        self.goal_theta = theta_rg
        fx = round(f * np.cos(ad_rg_h), 3)
        fy = round(f * np.sin(ad_rg_h), 3)
        self.target_f = (fx, fy)
        # self.target_f = (0.001, 0.001) # todel todo

    def f_robots(self):
        fx, fy = 0.0, 0.0
        new_robots = self.apf_robots
        for robot in new_robots:
            if not robot.cluster:
                force = self.compute_robot_force(robot)
            elif self.use_fake_cluster_obsts:
                # if (not self.near_robots) and (not self.near_obst):
                force = self.compute_multi_force(robot)

            fx += force[0]
            fy += force[1]

        coeff_f = 1
        fx = round(fx * coeff_f, 3)
        fy = round(fy * coeff_f, 3)
        self.robot_f = (fx, fy)

    def compute_multi_force(self, robo: ApfRobot) -> Tuple[float, float]:
        if self.near_robots or self.near_obst:
            return (0.0, 0.0)

        F_f = self.compute_robot_force(robo)
        return F_f

    def compute_robot_force(self, robo: ApfRobot) -> Tuple[float, float]:
        if robo.d > robo.r_start:
            return (0.0, 0.0)

        # near_robots
        if robo.d < robo.r_prec:
            self.near_robots = True

        # radial force
        f = ((robo.z * 1) * ((1 / robo.d) - (1 / robo.r_start))**2) * (1 / robo.d)**2
        fr = f + 0
        F_r = (fr * -np.cos(robo.ad_h_rR), fr * np.sin(robo.ad_h_rR))

        # [case 1] other Robot is not between robot and goal
        # goal-robot-Robot angle
        if abs(robo.ad_rg_rR) > (np.pi/2):
            # robot_between = False
            return F_r

        # [case 2] other Robot heading outward from robot
        ad_Rr_H = cal_angle_diff((robo.theta_rR - np.pi), robo.H)
        if abs(ad_Rr_H) > (np.pi/2):
            # Robot_away = True
            return F_r

        # c_t, for tangential force direction
        c_t = 1
        theta_ = np.deg2rad(10)  # todo param
        if abs(robo.ad_h_rR) < theta_:
            c_t = np.sign(robo.ad_rg_rR*robo.ad_h_rR)

        # tangential force
        theta_t = robo.theta_rR + (np.pi/2)*np.sign(robo.ad_h_rR)*c_t
        ad_t_h = cal_angle_diff(theta_t, self.pose.theta)
        ft = f + 3
        F_t = (ft * np.cos(ad_t_h), ft * np.sin(ad_t_h))
        F_f = (F_r[0]+F_t[0], F_r[1]+F_t[1])

        # [case 3] other Robot reached or stopped
        if robo.reached or robo.stopped:
            return F_f

        # check robot headings relative to the line rR
        ad_rR_h = -robo.ad_h_rR
        if (ad_Rr_H*ad_rR_h) < 0:
            if robo.prior:
                self.stop_flag_robots = True

        return F_f

    def f_obstacles(self):
        obs_f = [0, 0]
        self.obs_f = [0, 0]

        # real obstacles
        obst: Obstacle = None
        for i in self.f_obsts_inds:
            o_force = self.f_obstacle(self.obstacles[i])
            obs_f[0] += round(o_force[0], 3)
            obs_f[1] += round(o_force[1], 3)

        # fake obstacles - local minima avoidance
        obst: Obstacle = None
        for obst in self.fake_obsts_localmin:
            o_force = self.f_obstacle(obst)
            obs_f[0] += round(o_force[0], 3)
            obs_f[1] += round(o_force[1], 3)

        #
        coeff_f = 1
        self.obs_f[0] += round(obs_f[0] * coeff_f, 3)
        self.obs_f[1] += round(obs_f[1] * coeff_f, 3)

    def f_obstacle(self, obst: Obstacle):
        # robot-obstacle and heading angle differences
        dy = obst.y - self.pose.y
        dx = obst.x - self.pose.x
        d_ro = np.sqrt(dx**2 + dy**2)
        theta_ro = np.arctan2(dy, dx)
        ad_h_ro = cal_angle_diff(self.pose.theta, theta_ro)

        # check if too close to obstacle
        if d_ro < obst.obst_prec_d:
            self.near_obst = True

        # radial force
        f = ((obst.obst_z * 1) * ((1 / d_ro) - (1 / obst.obst_start_d))**2) * (1 / d_ro)**2
        F_r = (f * -np.cos(ad_h_ro), f * np.sin(ad_h_ro))

        # [case 1] if obstacle is not between robot and goal
        # goal-robot-obstacle angle
        ad_rg_ro = cal_angle_diff(self.theta_rg, theta_ro)
        if abs(ad_rg_ro) > (np.pi/2):
            return F_r

        # direction of the tangential force
        c_t = 1
        theta_ = np.deg2rad(20)  # todo param
        if abs(ad_h_ro) < theta_:
            c_t = np.sign(ad_rg_ro*ad_h_ro)
        theta_t = theta_ro + (np.pi/2)*np.sign(ad_h_ro)*c_t
        ad_t_h = cal_angle_diff(theta_t, self.pose.theta)

        # f tangential
        ft = f + 3
        F_t = (ft * np.cos(ad_t_h), ft * np.sin(ad_t_h))

        # apply tangential force
        F_f = (F_t[0]+F_r[0], F_t[1]+F_r[1])

        return F_f

    def calculate_velocity(self):
        f_r, f_theta = self.f_r, self.f_theta

        # v
        if f_r < 0:
            v = 0
        else:
            v = 1 * self.params.v_max * ((f_r / self.params.fix_f)**2)  # + self.params.v_min_2
        # w
        w = 5 * self.params.w_max * f_theta / self.params.fix_f
        if f_r < -1 and abs(w) < 0.05:
            w = 1*np.sign(w)

        # v
        if (v == 0) and abs(w) < 0.03:
            v = self.params.v_min_2*1

        # check bounds
        v = min(v, self.params.v_max)
        v = max(v, self.params.v_min)
        w = min(w, self.params.w_max)
        w = max(w, self.params.w_min)
        self.v = v
        self.w = w
