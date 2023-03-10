#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization import Viusalize
from apf.srv import SharePoses2, SharePoses2Request
from tf.transformations import euler_from_quaternion
# from shapely.geometry.polygon import Polygon
from shapely.geometry import Point, shape, MultiPoint


class NewRobots:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 1
        self.d = 0
        self.t = 0
        self.h_t = 0
        self.theta = 0
        self.r_prec = 0
        self.r_half = 0
        self.r_start = 0
        self.p = False
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
        self.goal_distance = 1000
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
        self.w_min = 0          # init_params.angular_min_speed
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
        # move
        self.go_to_goal()
        self.is_reached = True
        return

    # --------------------------  go_to_goal  ---------------------------#

    def go_to_goal(self):
        while self.goal_distance > self.dis_tresh and not rospy.is_shutdown():

            # detect and group
            self.detect_group()

            if  self.stop_flag_0:
                self.v = 0
                self.w = 0
                req = SharePoses2Request()
                req.ind = self.ind
                req.stopped = True
                self.pose_client(req)
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

            if self.ind==1: print("f_r", round(f_r, 2), "f_theta", round(f_theta, 2))
            if self.ind==1: print("moving", "v", round(self.v, 2), "w", round(self.w, 2))
            if self.ind==1: print(" ---------------------------------- ")
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
        v = 1 * self.v_max * (1-abs(theta)/thresh_theta)

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

        robots_inds = []
        robots_theta = []
        new_robots = []
        self.new_robots = []

        mp_bound = []
        is_in_t = False

        self.is_multi = False
        self.is_robots = False
        self.stop_flag_0 = False

        # get data
        req_poses2 = SharePoses2Request()
        req_poses2.update = False
        req_poses2.ind = self.ind
        req_poses2.stopped = False
        resp_poses2 = self.pose_client(req_poses2)
        robots_x = resp_poses2.x
        robots_y = resp_poses2.y
        robots_h = resp_poses2.heading
        robots_stopped = resp_poses2.stopp
        robots_reached = resp_poses2.reached
        robots_priority = resp_poses2.priority

        # get indices of robots in proximity circle
        polys = []
        polys0 = []
        for i in range(resp_poses2.count):
            flag_1 = False
            flag_2 = False
            dx = (robots_x[i] - self.r_x)
            dy = (robots_y[i] - self.r_y)
            d_rr = np.sqrt(dx**2 + dy**2)
            if d_rr < 2 * self.robot_start_d:   #####
                theta = np.arctan2(dy, dx)
                angle_diff_r = self.r_theta - theta
                angle_diff_r0 = (np.arctan2(np.sin(angle_diff_r), np.cos(angle_diff_r)))
                angle_diff_r = abs(angle_diff_r0)
                angle_diff_rr = (robots_h[i] - (theta - np.pi))
                angle_diff_rr = abs(np.arctan2(np.sin(angle_diff_rr), np.cos(angle_diff_rr)))

                # and (not robots_stopped[i])
                if (not robots_reached[i]) and (angle_diff_r < np.pi / 2 and angle_diff_rr < np.pi / 2): #and abs(angle_diff_r + angle_diff_rr) < np.pi / 2:  ##############
                    flag_1 = True
                if (not robots_reached[i]) and (angle_diff_r < np.pi / 2 and angle_diff_rr < np.pi / 2):
                    flag_2 = True

                if flag_1:
                    robots_theta.append(theta)
                    robots_inds.append(i)
                    polys.append((robots_x[i], robots_y[i]))
                if flag_2:
                    polys0.append((robots_x[i], robots_y[i]))
                

            if d_rr < 1 * self.robot_start_d:
                nr = NewRobots()
                nr.d = d_rr
                nr.theta = theta
                nr.x = robots_x[i]
                nr.y = robots_y[i]
                nr.t = robots_h[i]
                nr.h_t = angle_diff_r0
                nr.p = robots_priority[i]
                nr.r_prec = self.robot_prec_d
                nr.r_half = self.robot_half_d
                nr.r_start = self.robot_start_d
                nr.z = 4 * self.fix_f * nr.r_prec**4
                new_robots.append(nr)

        # if there is none robots in proximity
        if len(new_robots)==0:
            return


        if len(polys0)>2:
            point_t = Point(self.goal_x, self.goal_y)
            point_r = Point(self.r_x, self.r_y)
            points = [Point(p) for p in  polys0]
            mpt = MultiPoint([shape(p) for p in points])
            mp = mpt.convex_hull
            mp_bound = mp.boundary.coords
            is_in_t = mp.contains(point_t)
            is_in = mp.contains(point_r)
            if (not is_in_t) and is_in:
                self.stop_flag_0 = True

        self.is_robots = True
        self.new_robots = new_robots

        polys_i = []
        if (not is_in_t) and (not len(robots_inds)<2):
            # detect arc (poly)
            self.is_multi = True
            a_min = min(robots_theta)
            a_max = max(robots_theta)
            a_mean = (a_min+a_max)/2.0 
            self.multi_theta = a_min-np.pi/2 # a_min a_mean
            i_min = np.argmin(robots_theta)
            i_max = np.argmax(robots_theta)
            polys_i = [polys[i_min], polys[i_max]]
            polys_i.append((self.r_x, self.r_y))

        
    
        self.vs.robot_poly([polys_i, mp_bound], self.ns)

    # -----------------------  f_target  ----------------------------#

    def f_target(self):
        dx = self.goal_x - self.r_x
        dy = self.goal_y - self.r_y
        goal_distance = np.sqrt(dx**2 + dy**2)
        # f = self.zeta * goal_distance
        f = self.fix_f
        theta = np.arctan2(dy, dx)
        angle_diff = theta - self.r_theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        self.goal_theta = theta
        self.goal_distance = goal_distance
        fx = round(f * np.cos(angle_diff), 3)
        fy = round(f * np.sin(angle_diff), 3)
        self.target_f = [fx, fy]

    # -----------------------  f_robots  ----------------------------#

    def f_robots(self):

        robot_flag = False
        self.stop_flag = False
        robot_f = [0, 0]
        self.robot_f = [0,0]
        new_robots = self.new_robots

        # if new_robots==[]:
        #     return

        # ------------------
        for nr in new_robots:
            if (nr.d< nr.r_start):
                robot_flag = True
                if (nr.d< nr.r_prec) and (abs(nr.h_t)<np.pi/2):
                    self.stop_flag = True

                # compute force
                angle_diff = nr.h_t
                f = ((nr.z * 1) * ((1 / nr.d) - (1 / nr.r_start))**2) * (1 / nr.d)**2
                templ = [f * -np.cos(angle_diff), f * np.sin(angle_diff)]

                # adjust heading
                if (nr.r_prec<nr.d<nr.r_start):      # r_start r_half
                    if (abs(angle_diff)<np.pi/2):
                        coeff_alpha = np.cos(angle_diff)
                        # goal_theta = self.mod_angle(self.goal_theta)
                        # angle_diff4 = goal_theta - nr.theta
                        # angle_diff4 = np.arctan2(np.sin(angle_diff4), np.cos(angle_diff4))
                        # if angle_diff4*angle_diff<0:
                        #     coeff_alpha = -1*(coeff_alpha+1)
                        templ[1] = (f+3.0)*coeff_alpha*np.sign(angle_diff)
                    # else:
                    #     templ[0] = 3
                    #     templ[1] = 0
                # elif nr.d<nr.r_prec:
                #     if (abs(angle_diff)>np.pi/2):
                #         templ[0] = f
                        # templ[1] = 0

                robot_f[0] += round(templ[0], 3)
                robot_f[1] += round(templ[1], 3)

        if self.is_multi:  # (not robot_flag) and
            f = 8
            angle_diff = self.multi_theta - self.r_theta
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
            if angle_diff<0:
                templ = [f * np.cos(angle_diff), f * np.sin(angle_diff)]
                robot_f[0] += round(templ[0], 3)
                robot_f[1] += round(templ[1], 3)

        coeff_f = 1
        self.robot_f[0] += round(robot_f[0] * coeff_f, 3)
        self.robot_f[1] += round(robot_f[1] * coeff_f, 3)

    # -----------------------  f_obstacle  ----------------------------#

    def f_obstacle(self):
        obst_flag = False
        self.obs_f = [0, 0]
        obs_f = [0, 0]
        for i in self.obs_ind_main:
            dy = (self.obs_y[i] - self.r_y)
            dx = (self.obs_x[i] - self.r_x)
            d_ro = np.sqrt(dx**2 + dy**2)

            if d_ro > self.obst_start_d:
                continue

            obst_flag = True
            theta = np.arctan2(dy, dx)
            angle_diff = self.r_theta - theta
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

            f = ((self.obst_z * 1) * ((1 / d_ro) - (1 / self.obst_start_d))**2) * (1 / d_ro)**2
            templ = [f * -np.cos(angle_diff), f * np.sin(angle_diff)]

            if (self.obst_prec_d<d_ro<self.obst_start_d):  #obst_half_d
                if (abs(angle_diff)<np.pi/2):
                    coeff_alpha = np.cos(angle_diff)
                    goal_theta = self.mod_angle(self.goal_theta)
                    angle_diff4 = (theta-np.pi) - goal_theta
                    angle_diff4 = np.arctan2(np.sin(angle_diff4), np.cos(angle_diff4))
                    if angle_diff4*angle_diff<0:
                        coeff_alpha = -1*coeff_alpha
                    templ[1] = (f+3)*coeff_alpha*np.sign(angle_diff)
                else:
                    templ[0] = f+3
                    # templ[1] = 0
            # elif d_ro>self.obst_half_d:
            #     if (abs(angle_diff)>np.pi/2):
            #         templ[0] = 0
            #         templ[1] = 0

            obs_f[0] += round(templ[0], 3)
            obs_f[1] += round(templ[1], 3)

        coeff_f = 1
        if obst_flag:
            # abst_f = np.sqrt((obs_f[0]**2 + obs_f[1]**2))
            # if abst_f>0:
            #     coeff_f = min(abst_f, self.fix_f2) / abst_f

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
        self.r_theta = orientation[2]
        return self.topic_msg

    def get_odom(self, odom):
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.r_x = position.x
        self.r_y = position.y
        self.r_theta = orientation[2]

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