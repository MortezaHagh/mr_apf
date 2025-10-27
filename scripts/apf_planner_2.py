#! /usr/bin/env python

from typing import List, Dict, Tuple
import numpy as np
from shapely.coords import CoordinateSequence
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
        self.lg = MyLogger(f"APFPlanner2_r{robot.rid}")
        self.use_fake_cluster_obsts = False

        #
        self.apf_robots: List[ApfRobot] = []
        self.fake_obsts_localmin: List[Obstacle] = []
        self.multi_robots_vis: List[ApfRobot] = []
        self.clusters_x_y: List[Tuple] = []

        #
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_robot_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False
        self.stop_flag_multi = False

        # config
        self.c_radius = 2.5
        self.c_r_R_cluster = 2.0
        self.c_r_goal_cluster = 2.0
        self.ad_h_rR_thresh = np.deg2rad(10)
        self.ad_h_ro_thresh = np.deg2rad(20)

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

        # detect obstacles
        self.detect_obstacles_in_proximity()

        # detect and group
        self.create_all_fake_obstacles()

        # check stop_flag_multi
        if self.stop_flag_multi:
            self.lg.warn("stop_flag_multi activated.")
            self.stopped = True
            self.v = 0
            return False
        else:
            # calculate forces
            self.calculate_forces()

            # check stop flags
            if self.stop_flag_obsts or self.stop_flag_robots:
                self.lg.warn("stop_flag_obsts or stop_flag_robots activated.")
                self.stopped = True
                self.v = 0
                if abs(self.w) < self.params.w_stall:
                    self.v = self.params.v_min_2
                return False

            # check stop_flag_robot_full
            if self.stop_flag_robot_full:
                self.lg.warn("stop_flag_robot_full activated.")
                self.stopped = True
                self.v = 0
                # self.w = 0
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
        multi_robots_vis: List[ApfRobot] = []

        # is_goal_close
        goal_dist = cal_distance(self.pose.x, self.pose.y, self.goal_x, self.goal_y)
        if goal_dist < (self.c_r_goal_cluster*self.params.robot_d_start):
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
                multi_robots_vis.append(multi_robots[0])
                apf_robots.append(multi_robots[0])

        #
        self.multi_robots_vis = multi_robots_vis
        self.apf_robots = apf_robots
        return

    def detect_obstacles_in_proximity(self):
        """ detect obstacles in proximity circle"""
        f_obsts_inds = []
        for i, obst in enumerate(self.obstacles):
            do = cal_distance(obst.x, obst.y, self.pose.x, self.pose.y)
            if do < obst.d_start:
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

        #
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
            if d_rR > (self.c_r_R_cluster * self.params.robot_d_start):
                continue

            # calculate angle difference
            theta_rR = np.arctan2(dy, dx)
            ad_h_rR = cal_angle_diff(self.pose.theta, theta_rR)
            AD_h_rR[orob.rid] = ad_h_rR
            # ad_h_rR_abs = abs(ad_h_rR)
            # ad_H_Rr = cal_angle_diff(orob.h, (theta_rR - np.pi))
            # ad_H_Rr_abs = abs(ad_H_Rr)

            # if (not orob.reached) or (d_rR < (1 * self.params.robot_d_start)):
            # if (d_rR < (1 * self.params.robot_d_start)) or ((not orob.reached) or (ad_h_rR_abs < np.pi/2 or ad_H_Rr_abs < np.pi/2)):

            # check if robot is not reached, add to clust_candid_inds
            if not orob.reached:
                clust_candid_inds.append(orob.rid)

            # create ApfRobot instances as individual robots
            if d_rR < (1 * self.params.robot_d_start):
                new_robot = self.create_new_apf_robot(orob, d_rR, ad_h_rR, theta_rR)
                apf_robots.append(new_robot)

        #
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
                if d_Ro < (orob.r_prec + obst.d_prec) and d_Ro > (orob.r_prec + obst.d_prec)*0.75:
                    x = (xo+orob.x)/2
                    y = (yo+orob.y)/2
                    r = d_Ro/3
                    fake_obsts.append(Obstacle(x=x, y=y, r=r, params=self.params))
        self.fake_obsts_localmin = fake_obsts

    def create_fake_obst_clusters(self, clust_candid_inds: List[int], AD_h_rR: Dict[int, float]) -> List[ApfRobot]:
        """ create fake obstacles from clustering
        """

        #
        robots_inds_f = {}
        groups = []
        multi_robots: List[ApfRobot] = []

        # fleet data
        fd: FleetData = self.fleet_data

        # generate robots_inds_f (neighbor robots in proximity circle)
        robots_inds_2 = clust_candid_inds[:]
        while (len(robots_inds_2) > 0):
            p = robots_inds_2.pop(0)
            robots_inds_f[p] = [p]
            if len(robots_inds_2) == 0:
                break
            for ind_j in robots_inds_2:
                # if not (fd.fdata[p].reached and fd.fdata[ind_j].reached):
                dist = cal_distance(fd.fdata[p].x, fd.fdata[p].y, fd.fdata[ind_j].x, fd.fdata[ind_j].y)
                if (dist < (self.params.robot_d_prec*2.2)):    # param 2
                    robots_inds_f[p].append(ind_j)

        # detect groups
        robots_inds_3 = clust_candid_inds[:]
        while len(robots_inds_3) > 0:
            groups.append([])
            p = robots_inds_3.pop(0)
            gset = set(robots_inds_f[p])
            robots_inds_4 = robots_inds_3[:]
            for ind_j in robots_inds_4:
                nset = set(robots_inds_f[ind_j])
                if len(gset.intersection(nset)) > 0:
                    gset = gset.union(nset)
                    robots_inds_3.remove(ind_j)
            groups[-1] = list(gset)

        # groups - apf_robots -----------------------------------
        for g in groups:
            if len(g) > 1:
                fake_r = ApfRobot()
                fake_r.cluster = True
                is_robot_in = False
                is_target_in = False

                # priorities
                P = [fd.fdata[i].priority > self.robot_data.priority for i in g]

                # polygon
                is_g2 = True
                if len(g) > 2:
                    point_robot = Point(self.pose.x, self.pose.y)
                    point_target = Point(self.goal_x, self.goal_y)
                    polys_points = [Point(fd.fdata[i].x, fd.fdata[i].y) for i in g if (not fd.fdata[i].reached)]
                    if (len(polys_points) > 2):
                        is_g2 = False
                        mpt = MultiPoint([shape(p) for p in polys_points])
                        mp = mpt.convex_hull
                        cluster_poly_coords: CoordinateSequence = mp.boundary.coords
                        # mpc = mp.centroid.coords[0]
                        is_robot_in = mp.contains(point_robot)
                        is_target_in = mp.contains(point_target)
                        self.clusters_x_y.append(cluster_poly_coords.xy)

                        # get the minimum bounding circle of the convex hull
                        mbr = mp.minimum_rotated_rectangle
                        circum_center = mbr.centroid.coords[0]
                        # calculate the radius of the circumscribed circle
                        radius = mbr.exterior.distance(mbr.centroid)
                        xc = circum_center[0]
                        yc = circum_center[1]
                        rc = self.c_radius*radius  # + self.params.robot_r + self.params.d_prec    # param 3
                        rc = max(rc, 2*self.params.robot_d_prec)

                # if robot is in the polygon
                if (not is_target_in) and is_robot_in:
                    self.stop_flag_multi = True
                    return multi_robots

                # robot is not in the polygon, detect """triangle"""
                if is_g2:
                    ad = [AD_h_rR[i] for i in g]
                    a_min = g[np.argmin(ad)]
                    a_max = g[np.argmax(ad)]

                    x1 = fd.fdata[a_min].x
                    x2 = fd.fdata[a_max].x
                    y1 = fd.fdata[a_min].y
                    y2 = fd.fdata[a_max].y
                    dx = x2-x1
                    dy = y2-y1
                    theta = np.arctan2(dy, dx)
                    d12 = cal_distance(x1, y1, x2, y2)

                    xx1 = x1 + (d12/np.sqrt(3)) * np.cos(theta+np.pi/6)
                    yy1 = y1 + (d12/np.sqrt(3)) * np.sin(theta+np.pi/6)
                    xx2 = x1 + (d12/np.sqrt(3)) * np.cos(theta-np.pi/6)
                    yy2 = y1 + (d12/np.sqrt(3)) * np.sin(theta-np.pi/6)
                    dd1 = cal_distance(xx1, yy1, self.pose.x, self.pose.y)
                    dd2 = cal_distance(xx2, yy2, self.pose.x, self.pose.y)
                    if (dd1 < dd2):
                        xc = xx2
                        yc = yy2
                    else:
                        xc = xx1
                        yc = yy1
                    rc = d12/np.sqrt(2)      # /np.sqrt(3) d12 #$

                #
                d_tc = cal_distance(self.goal_x, self.goal_y, xc, yc)
                if (d_tc < rc):
                    is_target_in = True
                    continue

                # r_c, r_R
                dx = xc - self.pose.x
                dy = yc - self.pose.y
                d_rR = np.sqrt(dx**2 + dy**2)
                theta_rR = np.arctan2(dy, dx)
                ad_h_rR = cal_angle_diff(self.pose.theta, theta_rR)

                fake_r.x = xc
                fake_r.y = yc
                fake_r.d = d_rR
                fake_r.ad_h_rR = ad_h_rR
                fake_r.theta_rR = theta_rR
                if any(P):
                    fake_r.prior = True
                fake_r.r_prec = rc + self.params.robot_r + self.params.d_prec
                # fake_r.r_prec = self.eval_obst(xc, yc, fake_r.r_prec, d_rR)
                fake_r.r_half = 1.5 * fake_r.r_prec
                fake_r.r_start = 2.0 * fake_r.r_prec
                fake_r.z = 4 * self.params.fix_f * fake_r.r_prec**4
                # apf_robots.append(fake_r)
                multi_robots.append(fake_r)

        if len(multi_robots) > 1:
            rcs = [mr.r_prec for mr in multi_robots]
            rcs = np.array(rcs)
            max_ind = np.argmax(rcs)
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
        r_prec = self.params.robot_d_prec
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
        force = (0, 0)
        if (robo.r_start < robo.d):
            return force

        # force
        coeff = 1
        f1 = ((robo.z * 1) * ((1 / robo.d) - (1 / robo.r_start))**2) * (1 / robo.d)**2
        f = f1 + 2
        F_r = (f * -np.cos(robo.ad_h_rR), f * np.sin(robo.ad_h_rR))
        F_f = F_r

        # based_on_goal, target_other_side
        based_on_goal = False
        dx = self.goal_x - robo.x
        dy = self.goal_y - robo.y
        theta_Rg = np.arctan2(dy, dx)
        theta_Rr = robo.theta_rR - np.pi
        ad_Rg_Rr = cal_angle_diff(theta_Rg, theta_Rr)
        if abs(ad_Rg_Rr) < np.deg2rad(180-20):
            based_on_goal = True
        target_other_side = False
        if abs(ad_Rg_Rr) > np.pi/5:
            target_other_side = True

        theta_ = 20
        ad_rg_rR = cal_angle_diff(self.theta_rg,  robo.theta_rR)
        if based_on_goal:
            coeff = np.sign(ad_rg_rR*robo.ad_h_rR)
        else:
            if abs(robo.ad_h_rR) < np.deg2rad(theta_):
                coeff = np.sign(ad_rg_rR*robo.ad_h_rR)

        angle_turn_r = robo.theta_rR + (np.pi/2+np.pi/8)*np.sign(robo.ad_h_rR)*coeff
        ad_c_h = cal_angle_diff(angle_turn_r, self.pose.theta)
        f3 = f1 + 4
        templ3 = (f3 * np.cos(ad_c_h), f3 * np.sin(ad_c_h))

        if target_other_side:
            if (robo.r_prec < robo.d):
                F_f = templ3
            elif (0.8*robo.r_prec < robo.d < robo.r_prec):
                F_f = templ3

        return F_f

    def compute_robot_force(self, robo: ApfRobot) -> Tuple[float, float]:
        if robo.d > robo.r_start:
            return (0.0, 0.0)

        # near_robots
        if robo.d < robo.r_half:
            self.near_robots = True

        # radial force
        f = ((robo.z * 1) * ((1 / robo.d) - (1 / robo.r_start))**2) * (1 / robo.d)**2
        fr = f + 0
        F_r = (fr * -np.cos(robo.ad_h_rR), fr * np.sin(robo.ad_h_rR))

        # [case 1] other Robot is not between robot and goal
        # goal-robot-Robot angle
        ad_rg_rR = cal_angle_diff(self.theta_rg, robo.theta_rR)
        if abs(ad_rg_rR) > (np.pi/2):
            # robot_between = False
            return F_r

        # [case 2] other Robot heading outward from robot
        ad_Rr_H = cal_angle_diff((robo.theta_rR - np.pi), robo.H)
        if abs(ad_Rr_H) > (np.pi/2):
            # Robot_away = True
            return F_r

        # c_t, for tangential force direction
        c_t = 1
        if abs(robo.ad_h_rR) < self.ad_h_rR_thresh:
            c_t = np.sign(ad_rg_rR*robo.ad_h_rR)

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
        ad_rR_h = np.pi-robo.ad_h_rR
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
        if d_ro < obst.d_half:
            self.near_obst = True

        # radial force
        f = ((obst.obst_z * 1) * ((1 / d_ro) - (1 / obst.d_start))**2) * (1 / d_ro)**2
        F_r = (f * -np.cos(ad_h_ro), f * np.sin(ad_h_ro))

        # [case 1] if obstacle is not between robot and goal
        # goal-robot-obstacle angle
        ad_rg_ro = cal_angle_diff(self.theta_rg, theta_ro)
        if abs(ad_rg_ro) > (np.pi/2):
            return F_r

        # direction of the tangential force
        c_t = 1
        if abs(ad_h_ro) < self.ad_h_ro_thresh:
            c_t = np.sign(ad_rg_ro*ad_h_ro)
        theta_t = theta_ro + (np.pi/2)*np.sign(ad_h_ro)*c_t
        ad_t_h = cal_angle_diff(theta_t, self.pose.theta)

        # f tangential
        ft = f + 3
        F_t = (ft * np.cos(ad_t_h), ft * np.sin(ad_t_h))

        # apply tangential force
        F_f = (F_t[0]+F_r[0], F_t[1]+F_r[1])
        # if d_ro > obst.d_prec and d_ro < obst.d_half:
        #     F_f = (F_t[0]+F_r[0], F_t[1]+F_r[1])
        # elif (obst.d_half < d_ro):
        #     if (abs(ad_h_ro) < np.pi/2):
        #         F_f = (F_t[0]+F_r[0], F_t[1]+F_r[1])

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
