#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization import Viusalize
from apf.srv import SharePoses2, SharePoses2Request
from tf.transformations import euler_from_quaternion

from shapely.geometry.polygon import Polygon
from shapely.geometry import Point, shape, MultiPoint


class NewRobots:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 1
        self.d = 0
        self.H = 0
        self.h_rR = 0
        self.theta_rR = 0
        self.r_prec = 0
        self.r_half = 0
        self.r_start = 0
        self.p = False
        self.stop = False
        self.reached = False
        self.big = False


class ApfMotion(object):

    def __init__(self, model, robot, init_params):

        # ros
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown_hook)

        # Viusalize
        self.vs = Viusalize(model)

        # preallocation and params and setting
        self.init(model, robot, init_params)

        # map: target and obstacles coordinates
        self.map()

        # /cmd_vel puplisher
        self.cmd_vel = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)

        # listener
        self.check_topic()
        rospy.Subscriber(self.topic, self.topic_type, self.get_odom)

        # pose service client
        rospy.wait_for_service(self.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(self.pose_srv_name, SharePoses2)

        # execute goal
        self.exec_cb()

    # --------------------------  init  ---------------------------#

    def init(self, model, robot, init_params):
        # preallocation
        self.phis = []
        self.v_lin = []
        self.v_ang = []
        self.path_x = []
        self.path_y = []
        self.force_tr = []
        self.force_tt = []
        self.force_or = []
        self.force_ot = []
        self.stop_flag = False

        # data
        self.model = model
        self.robot = robot

        # parameters
        self.goal_theta = 0
        self.goal_dist = 1000
        self.topic_type = Odometry
        self.prioriy = robot.priority

        # params
        self.ind = init_params.id
        self.ns = init_params.name_space
        self.topic = init_params.lis_topic
        self.cmd_topic = init_params.cmd_topic
        self.pose_srv_name = init_params.pose_srv_name

        # parameters vel
        self.v = 0
        self.w = 0
        self.v_max = 0.2        # init_params.linear_max_speed
        self.v_min = 0.0        # init_params.linear_min_speed
        self.w_min = 0.0        # init_params.angular_min_speed
        self.w_max = 1.0        # init_params.angular_max_speed
        self.v_min_2 = 0.04     # init_params.linear_min_speed_2

        # settings
        self.zeta = 1
        self.fix_f = 4
        self.fix_f2 = 10
        self.obst_r = 0.11
        self.prec_d = 0.06
        self.robot_r = 0.22

        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_half_d = 1.5*self.obst_prec_d
        self.obst_start_d = 2*self.obst_prec_d
        self.obst_z = 4*self.fix_f*self.obst_prec_d**4

        self.robot_prec_d = 2*self.robot_r + self.prec_d  # 0.64
        self.robot_start_d = 2*self.robot_prec_d
        self.robot_half_d = 1.5*self.robot_prec_d
        self.robot_stop_d = self.robot_prec_d
        self.robot_z = 4 * self.fix_f*self.robot_prec_d**4

        self.w_coeff = 1                        # init_params.w_coeff      # angular velocity coeff
        self.dis_tresh = init_params.dis_tresh  # distance thresh to finish
        self.theta_thresh = 30 * np.pi / 180    # init_params.theta_thresh  # for velocity calculation

    # --------------------------  exec_cb  ---------------------------#

    def exec_cb(self):
        self.go_to_goal()
        self.is_reached = True
        return

    # --------------------------  go_to_goal  ---------------------------#

    def go_to_goal(self):
        while self.goal_dist > self.dis_tresh and not rospy.is_shutdown():

            # detect and group
            self.detect_group()

            if self.stop_flag_multi:
                req = SharePoses2Request()
                req.ind = self.ind
                req.stopped = True
                self.pose_client(req)
                self.v = 0
                self.w = 0
            else:

                # calculate forces
                [f_r, f_theta, phi, stop_flag] = self.forces()

                # calculate velocities
                self.cal_vel(f_r, f_theta, phi)
                self.v_lin.append(self.v)
                self.v_ang.append(self.w)

                if stop_flag:
                    self.v = 0
                    # self.w = 0

            # publish cmd
            move_cmd = Twist()
            move_cmd.linear.x = self.v
            move_cmd.angular.z = self.w
            self.cmd_vel.publish(move_cmd)

            # result
            self.path_x.append(round(self.r_x, 3))
            self.path_y.append(round(self.r_y, 3))

            if self.ind==1: print("f0: ", self.stop_flag_multi, "f: ", self.stop_flag)
            if self.ind==1: print("f_r", round(f_r, 2), "f_theta", round(f_theta, 2))
            if self.ind==1: print("moving", "v", round(self.v, 2), "w", round(self.w, 2))
            if self.ind==1: print(" ------------------------------------ ")
            self.rate.sleep()

        req = SharePoses2Request()
        req.ind = self.ind
        req.reached = True
        self.pose_client(req)
        self.stop()

    # -----------------------  cal_vel  ----------------------------#

    def cal_vel(self, f_r, f_theta, theta):

        # if f_r < 0:
        #     v = 0
        # else:
        #     v = 1 * self.v_max * ((f_r / self.fix_f)**2 + (f_r / self.fix_f) / 4) + self.v_min_2

        # w = 1 * self.w_max * f_theta / self.fix_f

        # if (v==0) and abs(w)<0.03:
        #     v = self.v_min_2*2

        thresh_theta = np.pi/3
        w = 4 * self.w_max * theta / (np.pi/5)
        v = 2 * self.v_max * (1-abs(theta)/thresh_theta)

        if (v<self.v_min_2) and abs(w)<0.03:
            v = self.v_min_2*2

        v = min(v, self.v_max)
        v = max(v, self.v_min)
        wa = min(abs(w), self.w_max)
        w = wa * np.sign(w)

        self.v = v
        self.w = w

    # -----------------------  forces  ----------------------------#

    def forces(self):
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]

        # self.f_obstacle()
        # f_r += self.obs_f[0]
        # f_theta += self.obs_f[1]

        self.f_robots()
        f_r += self.robot_f[0]
        f_theta += self.robot_f[1]

        theta = np.arctan2(f_theta, f_r)
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        phi = round(theta, 4)

        # self.phis.append(phi)
        # self.force_or.append(self.obs_f[0])
        # self.force_ot.append(self.obs_f[1])
        # self.force_tr.append(self.target_f[0])
        # self.force_tt.append(self.target_f[1])

        return [f_r, f_theta, phi, self.stop_flag]

    # -----------------------  detect_group  ----------------------------#

    def detect_group(self):

        groups = []
        D_rR = []
        AD_h_rR = []
        THETA_rR = []
        new_robots = []
        multi_robots = []
        robots_inds = []
        robots_inds_f = {}
        self.new_robots = []

        is_goal_close = False
        self.stop_flag_multi = False

        self.f_obsts_inds = self.detect_obsts()

        # get data
        req_poses = SharePoses2Request()
        req_poses.ind = self.ind
        req_poses.update = False
        req_poses.stopped = False
        resp_poses = self.pose_client(req_poses)
        robots_x = resp_poses.x
        robots_y = resp_poses.y
        robots_h = resp_poses.heading
        robots_stopped = resp_poses.stopp
        robots_reached = resp_poses.reached
        robots_priority = resp_poses.priority

        goal_dist = self.distance(self.r_x, self.r_y, self.goal_x, self.goal_y)
        if (goal_dist < (2*self.robot_start_d)):
            is_goal_close = True

        # get indices of robots in proximity circle
        for i in range(resp_poses.count):
            dx = (robots_x[i] - self.r_x)
            dy = (robots_y[i] - self.r_y)
            d_rR = np.sqrt(dx**2 + dy**2)
            theta_rR = np.arctan2(dy, dx)
            ad_h_rR = self.angle_diff(self.r_h, theta_rR)
            THETA_rR.append(theta_rR)
            AD_h_rR.append(ad_h_rR)
            D_rR.append(d_rR)
            if (d_rR > (2 * self.robot_start_d)):   ##################
                continue
            robots_inds.append(i)
            
            # individual robots
            if (d_rR<(1 * self.robot_start_d)):
                    nr = NewRobots()
                    nr.d = d_rR
                    nr.x= robots_x[i]
                    nr.y= robots_y[i]
                    nr.H = robots_h[i]
                    nr.h_rR = ad_h_rR
                    nr.theta_rR = theta_rR
                    nr.p = robots_priority[i]>0
                    nr.stop = robots_stopped[i]
                    nr.reached = robots_reached[i]
                    rc = self.robot_prec_d
                    # rc = self.eval_obst(robots_x[i], robots_y[i], self.robot_prec_d)
                    nr.r_prec = rc
                    nr.r_half = 1.5 * rc
                    nr.r_start = 2.0 * rc
                    nr.z = 4 * self.fix_f * rc**4
                    new_robots.append(nr)
        
        # if there is none robots in proximity
        if len(robots_inds)==0:
            return
        
        # generate robots_inds_f (neighbor robots in proximity circle)
        robots_inds_2 = robots_inds[:]
        while (len(robots_inds_2)>0):
            p = robots_inds_2.pop(0)
            robots_inds_f[p] = [p]
            if len(robots_inds_2)==0:
                break
            for ind_j in robots_inds_2:
                dx = (robots_x[p] - robots_x[ind_j])
                dy = (robots_y[p] - robots_y[ind_j])
                dist = self.distance(robots_x[p], robots_y[p], robots_x[ind_j], robots_y[ind_j])
                if (dist<(self.robot_prec_d*2.5)):     ##### robot_start_d robot_prec_d
                    robots_inds_f[p].append(ind_j)

        # detect groups 
        robots_inds_3 = robots_inds[:]
        while len(robots_inds_3)>0:
            groups.append([])
            p = robots_inds_3.pop(0)
            gset = set(robots_inds_f[p])
            robots_inds_4 = robots_inds_3[:]
            for ind_j in robots_inds_4:
                nset = set(robots_inds_f[ind_j])
                if len(gset.intersection(nset))>0:
                    gset = gset.union(nset)
                    robots_inds_3.remove(ind_j)
            groups[-1] = list(gset)

        # groups - new_robots -----------------------------------
        for g in groups:
            if len(g)>1:
                nr = NewRobots()
                nr.big = True
                is_robot_in = False
                is_target_in = False

                # priorities
                P = [robots_priority[i]>0 for i in g]

                # polygon
                if len(g)>2:
                    point_robot = Point(self.r_x, self.r_y)
                    point_target = Point(self.goal_x, self.goal_y)
                    polys_points = [Point(robots_x[i], robots_y[i]) for i in g]
                    mpt = MultiPoint([shape(p) for p in polys_points])
                    mp = mpt.convex_hull
                    mp_bound = mp.boundary.coords
                    mpc = mp.centroid.coords[0]
                    is_robot_in = mp.contains(point_robot)
                    is_target_in = mp.contains(point_target)
                    self.vs.robot_poly([[], mp_bound], self.ns)

                
                # if robot is in the polygon
                if (not is_target_in) and is_robot_in:
                    self.stop_flag_multi = True
                    return
                
                # robot is not in the polygon, detect """triangle"""
                ad = [AD_h_rR[i] for i in g]
                a_min = g[np.argmin(ad)]
                a_max = g[np.argmax(ad)]

                x1 = robots_x[a_min]
                x2 = robots_x[a_max]
                y1 = robots_y[a_min]
                y2 = robots_y[a_max]
                dx = x2-x1
                dy = y2-y1 
                theta = np.arctan2(dy, dx)
                d12 = self.distance(x1, y1, x2, y2)

                xx1 = x1 + (d12/np.sqrt(3)) * np.cos(theta+np.pi/6) 
                yy1 = y1 + (d12/np.sqrt(3)) * np.sin(theta+np.pi/6)
                xx2 = x1 + (d12/np.sqrt(3)) * np.cos(theta-np.pi/6) 
                yy2 = y1 + (d12/np.sqrt(3)) * np.sin(theta-np.pi/6)
                dd1 = self.distance(xx1, yy1, self.r_x, self.r_y) 
                dd2 = self.distance(xx2, yy2, self.r_x, self.r_y)
                if (dd1<dd2):
                    xc = xx2
                    yc = yy2
                else:
                    xc = xx1
                    yc = yy1
                rc = d12/np.sqrt(3) # /np.sqrt(3) d12
                # rc = self.eval_obst(xc, yc, rc)

                #
                dx = xc - self.r_x
                dy = yc - self.r_y
                d_rR = np.sqrt(dx**2 + dy**2)
                theta_rR = np.arctan2(dy, dx)
                ad_h_rR = self.angle_diff(self.r_h, theta_rR)

                nr.x = xc
                nr.y = yc
                nr.d = d_rR
                nr.h_rR = ad_h_rR
                nr.theta_rR = theta_rR
                if any(P): nr.p = True
                nr.r_prec = rc + self.robot_r + self.prec_d
                nr.r_half = 1.5 * nr.r_prec
                nr.r_start = 2.0 * nr.r_prec
                nr.z = 4 * self.fix_f * nr.r_prec**4
                new_robots.append(nr)
                multi_robots.append(nr)
        
        self.new_robots = new_robots
        self.vs.robot_data(multi_robots, self.ns)
        return


    def detect_obsts(self):
        f_obsts_inds = []
        for oi in self.obs_ind_main:
            xo = self.obs_x[oi]
            yo = self.obs_y[oi]
            do = self.distance(xo, yo, self.r_x, self.r_y)
            if do<self.obst_start_d:
                f_obsts_inds.append(oi)
        return f_obsts_inds


    def eval_obst(self, xc, yc, rc):
        ros = []
        for oi in self.f_obsts_inds:
            xo = self.obs_x[oi]
            yo = self.obs_y[oi]
            do = self.distance(xo, yo, xc, yc)
            if rc<do<rc+self.obst_prec_d:
                ros.append(do)

        if ros!=[]:
            do_max = max(ros)
            rc = do_max
        return rc

    # -----------------------  f_target  ----------------------------#

    def f_target(self):
        dx = self.goal_x - self.r_x
        dy = self.goal_y - self.r_y
        goal_dist = np.sqrt(dx**2 + dy**2)
        # f = self.zeta * goal_dist
        f = self.fix_f
        theta = np.arctan2(dy, dx)
        angle_diff = theta - self.r_h
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        self.goal_theta = theta
        self.goal_dist = goal_dist
        fx = round(f * np.cos(angle_diff), 3)
        fy = round(f * np.sin(angle_diff), 3)
        self.target_f = [fx, fy]

    # -----------------------  f_robots  ----------------------------#

    def f_robots(self):
        
        robot_f = [0, 0]
        self.robot_f = [0,0]
        self.stop_flag = False
        new_robots = self.new_robots

        for nr in new_robots:
            if (not nr.big):
                templ = self.compute_robot_force(nr)
            
            # # check if in the circle and mowing towards it -> stop
            # if  (not nr.big) and (d_rR < 1.9*nr.r_prec) and (abs(angle_diff2) > np.pi/2): #****** idea
            #     if (nr.p): 
            #         self.stop_flag = True
            #     elif d_rR < 1.0*nr.r_prec:
            #         self.stop_flag = True

            else:
                f = ((nr.z * 1) * ((1 / nr.d) - (1 / nr.r_start))**2) * (1 / nr.d)**2
                f = f + 2
                templ = [f * -np.cos(nr.h_rR), f * np.sin(nr.h_rR)]

            robot_f[0] += round(templ[0], 3)
            robot_f[1] += round(templ[1], 3)

        coeff_f = 1
        self.robot_f[0] += round(robot_f[0] * coeff_f, 3)
        self.robot_f[1] += round(robot_f[1] * coeff_f, 3)

# -----------------------  compute_robot_force  ----------------------------#

    def compute_robot_force(self, nr):
        if (nr.d< nr.r_start):
            if (nr.d< nr.r_prec) and (abs(nr.h_rR)<np.pi/2):
                self.stop_flag = True

            #
            templ = []
            templ2 = []
            templ3 = []
            templ3_2 = []

            # compute force
            ad_h_rR = nr.h_rR
            ad_Rr_H = self.angle_diff((nr.theta_rR - np.pi), nr.H)
            angle_turn_R = nr.theta_rR - (np.pi/2)*np.sign(ad_Rr_H)
            ad_C_h = self.angle_diff(angle_turn_R, self.r_h)
            angle_turn_r = nr.theta_rR + (np.pi/2)*np.sign(ad_h_rR)
            ad_c_h = self.angle_diff(angle_turn_r, self.r_h)

            f = ((nr.z * 1) * ((1 / nr.d) - (1 / nr.r_start))**2) * (1 / nr.d)**2

            fl = f+2
            templ = [fl * -np.cos(ad_h_rR), fl * np.sin(ad_h_rR)]

            f2 = f + 2
            templ2 = [f2 * np.cos(ad_C_h), f2 * np.sin(ad_C_h)]

            f3 = fl
            templ3 = [f3 * np.cos(ad_c_h), f3 * np.sin(ad_c_h)]
            f3_2 = f + 2 
            templ3_2 = [f3_2 * np.cos(ad_c_h), f3_2 * np.sin(ad_c_h)]
            

            # adjust heading
            if (nr.r_half<nr.d<nr.r_start):
                if (not nr.reached) and (not nr.stop):
                    if (abs(ad_h_rR)<np.pi/2) and (abs(ad_Rr_H)<(np.pi/2)):
                        templ = [templ2[0]+templ[0], templ2[1]+templ[1]]
                else:
                    if (abs(ad_h_rR)<(np.pi/2)):
                        templ = [templ3[0]+templ[0], templ3[1]+templ[1]]

            elif (nr.r_prec <nr.d<nr.r_half):
                if (not nr.reached) and (not nr.stop):
                    if (abs(ad_Rr_H)<(np.pi/2)):
                        templ = [templ2[0]+templ[0], templ2[1]+templ[1]]
                else:
                    if (abs(ad_h_rR)<(np.pi/2)):
                        templ = [templ3_2[0]+templ[0], templ3_2[1]+templ[1]]
        return templ

    # -----------------------  f_obstacle  ----------------------------#

    def f_obstacle(self):
        obst_flag = False
        self.obs_f = [0, 0]
        obs_f = [0, 0]
        for i in self.obs_ind_main:
            dy = -(self.obs_y[i] - self.r_y)
            dx = -(self.obs_x[i] - self.r_x)
            d_ro = np.sqrt(dx**2 + dy**2)

            if d_ro > self.obst_start_d:
                continue
            
            obst_flag = True
            theta = np.arctan2(dy, dx)
            theta = self.mod_angle(theta)
            r_theta = self.mod_angle(self.r_h)
            angle_diff = theta - r_theta
            angle_diff2 = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

            f = ((self.obst_z * 1) * ((1 / d_ro) - (1 / self.obst_start_d))**2) * (1 / d_ro)**2
            templ = [f * np.cos(angle_diff2), f * np.sin(angle_diff2)]

            if (1.0*self.obst_prec_d<d_ro<2.0*self.obst_prec_d):
                if (abs(angle_diff2)>np.pi/2):
                    angle_diff3 = np.pi - abs(angle_diff2)
                    coeff_alpha = np.cos(angle_diff3)
                    # goal_theta = self.mod_angle(self.goal_theta)
                    # angle_diff4 = theta - goal_theta
                    # angle_diff4 = np.arctan2(np.sin(angle_diff4), np.cos(angle_diff4))
                    # if angle_diff4*angle_diff2<0:
                    #     coeff_alpha = -1*coeff_alpha
                    templ[1] = (f+3.2)*coeff_alpha*np.sign(np.sin(angle_diff2))

            #     else:
            #         templ[0] = f+3.5
            #         templ[1] = 0
            # else:
            #     if (abs(angle_diff2)<np.pi/2):
            #         templ[0] = f+3.5
            #         templ[1] = 0
            
            obs_f[0] += round(templ[0], 3)
            obs_f[1] += round(templ[1], 3)

        coeff_f = 1
        if obst_flag:
            self.obs_f[0] += round(obs_f[0] * coeff_f, 3)
            self.obs_f[1] += round(obs_f[1] * coeff_f, 3)

    # ------------------------- check_topic -- get_odom  ------------------------------------#

    def check_topic(self):
        self.topic_msg = None
        rospy.loginfo(self.ns + " apf_motion, checking topic ...")
        while self.topic_msg is None:
            try:
                self.topic_msg = rospy.wait_for_message(self.topic, self.topic_type, timeout=3.0)
                rospy.logdebug(self.ns + " apf_motion, current topic is ready!")
            except:
                rospy.loginfo(self.ns + " apf_motion, current topic is not ready yet, retrying ...")

        position = self.topic_msg.pose.pose.position
        quaternion = self.topic_msg.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.r_x = position.x
        self.r_y = position.y
        self.r_h = orientation[2]
        return self.topic_msg

    def get_odom(self, odom):
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.r_x = position.x
        self.r_y = position.y
        self.r_h = orientation[2]

    # ----------------  get_robot -- map -- modify_angle -- shutdown_hook -------------------#

    def map(self):

        # robot target
        self.goal_x = self.robot.xt
        self.goal_y = self.robot.yt

        # obstacles
        self.obs_count = self.model.obst.count
        self.obs_ind_main = [i for i in range(self.model.obst.count)]
        self.obs_ind = [i for i in range(self.model.obst.count)]
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y

    def modify_angle(self, theta):
        theta_mod = (theta + np.pi) % (2 * np.pi) - np.pi
        return theta_mod

    def stop(self):
        t = 0
        while t < 5:
            self.cmd_vel.publish(Twist())
            self.rate.sleep()
            t += 1

    def shutdown_hook(self):
        print("shutting down from apf_motion")
        self.stop()

    def mod_angle(self, theta):
        if theta < 0:
            theta = theta + 2 * np.pi
        elif theta > 2 * np.pi:
            theta = theta - 2 * np.pi
        return theta

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2+(y1-y2)**2)
    
    def angle_diff(self, a1, a2):
        ad = a1 - a2
        ad = np.arctan2(np.sin(ad), np.cos(ad))
        return ad