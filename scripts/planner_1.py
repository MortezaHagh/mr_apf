#! /usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from tf.transformations import euler_from_quaternion
from apf.srv import SharePoses2, SharePoses2Request, SharePoses2Response
from parameters import Params
from create_model import MRSModel
from visualization import RvizViusalizer
from my_utils import cal_angle_diff, cal_distance
from mrapf_classes import PlannerRobot, Records


class PlannerROS(object):
    model: MRSModel
    robot: PlannerRobot
    p: Params
    vs: RvizViusalizer

    def __init__(self, model: MRSModel, robot: PlannerRobot, params: Params):

        # initialize
        # data
        self.p = params
        self.model = model
        self.robot = robot
        self.map_data()         # map: target and obstacles coordinates
        self.pose = Pose2D()    # robot pose
        self.rec = Records()    # records

        #
        self.v = 0
        self.w = 0

        # compute vars
        self.ad_rg_h = None
        self.theta_rg = None
        self.goal_dist = None
        self.goal_theta = None
        #
        self.robot_f = []
        self.target_f = []
        self.obs_f = []

        # control vars
        self.is_multi = False
        self.near_obst = False
        self.near_robots = False
        self.stop_flag = False
        self.stop_flag_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False

        # control vars
        self.theta_rg = 0
        self.goal_theta = 0
        self.goal_dist = 1000

        # ---------------------------------------------------------------------

        # ros
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown_hook)

        # RvizViusalizer
        self.vs = RvizViusalizer(model)

        # /cmd_vel puplisher
        self.cmd_vel_pub = rospy.Publisher(self.p.cmd_topic, Twist, queue_size=5)

        # listener
        self.check_topic()
        rospy.Subscriber(self.p.lis_topic, Odometry, self.odom_cb)

        # pose service client
        rospy.wait_for_service(self.p.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(
            self.p.pose_srv_name, SharePoses2)

        # execute goal
        self.exec_cb()

    def exec_cb(self):
        self.go_to_goal()
        self.is_reached = True
        return

    def go_to_goal(self):
        while self.goal_dist > self.p.goal_dis_tresh and not rospy.is_shutdown():
            # calculate forces ================
            [f_r, f_theta, phi, stop_flag] = self.forces()

            # calculate velocities
            self.cal_vel(f_r, f_theta, phi)
            self.rec.v_lin.append(self.v)
            self.rec.v_ang.append(self.w)

            # check stop flags
            if stop_flag:
                self.v = 0.0
                self.w = 0.0

            # publish cmd
            move_cmd = Twist()
            move_cmd.linear.x = self.v
            move_cmd.angular.z = self.w
            self.cmd_vel_pub.publish(move_cmd)

            # record path
            self.rec.path_x.append(round(self.pose.x, 3))
            self.rec.path_y.append(round(self.pose.y, 3))

            #
            self.log_motion(f_r, f_theta)
            self.rate.sleep()

        # reached
        req = SharePoses2Request()
        req.id = self.p.id
        req.reached = True
        self.pose_client(req)
        self.stop()
        self.stop()

    def cal_vel(self, f_r, f_theta, theta):
        # v
        if f_r < 0:
            v = 0
        else:
            v = 1 * self.p.v_max * ((f_r / self.p.fix_f)**2) + self.p.v_min_2
        # w
        w = 3 * self.p.w_max * f_theta / self.p.fix_f
        if f_r < -1 and abs(w) < 0.05:
            w = 1*np.sign(w)

        # v
        if (v <= self.p.v_min_2*2) and abs(w) < 0.03:
            v = self.p.v_min_2*2

        # check bounds
        v = min(v, self.p.v_max)
        v = max(v, self.p.v_min)
        wa = min(abs(w), self.p.w_max)
        w = wa * np.sign(w)
        self.v = v
        self.w = w

    def forces(self):
        # target
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]
        # obstacles
        self.f_obstacle()
        f_r += self.obs_f[0]
        f_theta += self.obs_f[1]
        # robots
        self.f_robots()
        f_r += self.robot_f[0]
        f_theta += self.robot_f[1]
        # phi
        theta = np.arctan2(f_theta, f_r)
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        phi = round(theta, 4)
        # vizualize foice
        self.vizualize_force([f_r, f_theta], False)

        # self.rec.phis.append(phi)
        # self.rec.force_or.append(self.obs_f[0])
        # self.rec.force_ot.append(self.obs_f[1])
        # self.rec.force_tr.append(self.target_f[0])
        # self.rec.force_tt.append(self.target_f[1])

        return [f_r, f_theta, phi, self.stop_flag]

    def f_target(self):
        # r_g
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        goal_dist = np.sqrt(dx**2 + dy**2)
        f = self.p.zeta * goal_dist
        f = max(f, self.p.fix_f2)  # change
        # f = self.p.fix_f
        theta_rg = np.arctan2(dy, dx)
        ad_rg_h = cal_angle_diff(theta_rg, self.pose.theta)
        self.ad_rg_h = ad_rg_h
        self.theta_rg = theta_rg
        self.goal_dist = goal_dist
        self.goal_theta = theta_rg
        fx = round(f * np.cos(ad_rg_h), 3)
        fy = round(f * np.sin(ad_rg_h), 3)
        self.target_f = [fx, fy]

    def f_robots(self):
        robot_flag = False
        self.stop_flag = False
        req = SharePoses2Request()
        req.id = self.p.id
        req.update = False
        resp: SharePoses2Response = self.pose_client(req)
        robot_f = [0, 0]
        self.robot_f = [0, 0]
        #
        for i in range(resp.count):
            dx = -(resp.x[i] - self.pose.x)
            dy = -(resp.y[i] - self.pose.y)
            d_ro = np.sqrt(dx**2 + dy**2)
            theta = np.arctan2(dy, dx)
            angle_diff = cal_angle_diff(theta, self.pose.theta)

            if d_ro > 1 * self.p.robot_start_d:
                continue

            # and abs(angle_diff) > np.pi/2:
            if (not resp.reached) and d_ro < self.p.obst_half_d and resp.priority[i] > 0:
                self.stop_flag = True
                break

            robot_flag = True
            f = ((self.p.robot_z * 1) * ((1 / d_ro) -
                 (1 / self.p.robot_start_d))**2) * (1 / d_ro)**2
            templ = [f * np.cos(angle_diff), f * np.sin(angle_diff)]

            robot_f[0] += round(templ[0], 3)
            robot_f[1] += round(templ[1], 3)

        coeff_f = 1
        if robot_flag:
            self.robot_f[0] += round(robot_f[0] * coeff_f, 3)
            self.robot_f[1] += round(robot_f[1] * coeff_f, 3)

    def f_obstacle(self):
        obst_flag = False
        self.obs_f = [0, 0]
        obs_f = [0, 0]
        for i in range(self.obs_count):
            dy = -(self.obs_y[i] - self.pose.y)
            dx = -(self.obs_x[i] - self.pose.x)
            d_ro = np.sqrt(dx**2 + dy**2)

            if d_ro > self.p.obst_start_d:
                continue

            obst_flag = True
            theta = np.arctan2(dy, dx)
            angle_diff = theta - self.pose.theta
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

            f = ((self.p.obst_z * 1) * ((1 / d_ro) -
                 (1 / self.p.obst_start_d))**2) * (1 / d_ro)**2
            templ = [f * np.cos(angle_diff), f * np.sin(angle_diff)]

            obs_f[0] += round(templ[0], 3)
            obs_f[1] += round(templ[1], 3)

        coeff_f = 1
        if obst_flag:
            self.obs_f[0] += round(obs_f[0] * coeff_f, 3)
            self.obs_f[1] += round(obs_f[1] * coeff_f, 3)

    def check_topic(self):
        topic_msg: Odometry = None
        rospy.loginfo(self.p.ns + " apf_motion, checking topic ...")
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(self.p.lis_topic, Odometry, timeout=3.0)
                rospy.logdebug(self.p.ns + " apf_motion, current topic is ready!")
            except:
                rospy.loginfo(self.p.ns + " apf_motion, current topic is not ready yet, retrying ...")

        position = topic_msg.pose.pose.position
        quaternion = topic_msg.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.theta = orientation[2]

    def odom_cb(self, odom):
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.theta = orientation[2]

    def map_data(self):
        # robot target
        self.goal_x = self.robot.xt
        self.goal_y = self.robot.yt
        # obstacles
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y
        self.obs_count = self.model.obst.count

    def stop(self):
        t = 0
        while t < 5:
            self.cmd_vel_pub.publish(Twist())
            self.rate.sleep()
            t += 1

    def log_motion(self, f_r, f_theta):
        n = 1
        if self.p.id == n:
            print("f_r", round(f_r, 2), "f_theta", round(f_theta, 2))
            print("moving", "v", round(self.v, 2), "w", round(self.w, 2))
            print(" ------------------------------------ ")

    def shutdown_hook(self):
        print("shutting down from apf_motion")
        self.stop()

    def vizualize_force(self, nr_force, ip=True):
        theta = np.arctan2(nr_force[1], nr_force[0])
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        theta = self.pose.theta + theta
        if ip:
            self.vs.arrow(self.pose.x, self.pose.y, theta)
        else:
            self.vs.arrow(self.pose.x, self.pose.y, theta, self.p.ns)
