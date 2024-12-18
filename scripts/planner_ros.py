#! /usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from tf.transformations import euler_from_quaternion
from apf.srv import SharePoses2, SharePoses2Request, SharePoses2Response
from parameters import Params
from create_model import MRSModel
from apf_planner_2 import APFPlanner
from visualization import RvizViusalizer
from mrapf_classes import PlannerRobot, Records


class PlannerROS:
    rid: int
    ns: str
    p: Params
    pose: Pose2D
    ap: APFPlanner
    vs: RvizViusalizer

    def __init__(self, model: MRSModel, robot: PlannerRobot, params: Params):

        #
        self.v = 0
        self.w = 0
        self.rid = params.id
        self.ns = params.ns  # name space

        # apf planner
        self.ap = APFPlanner(model, robot, params)

        #
        self.pose = Pose2D()    # robot pose
        self.rec = Records()    # records

        # ros
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown_hook)

        # RvizViusalizer
        self.vs = RvizViusalizer(model)

        # /cmd_vel puplisher
        self.cmd_vel_pub = rospy.Publisher(params.cmd_topic, Twist, queue_size=5)

        # listener
        self.check_topic(params.lis_topic)
        rospy.Subscriber(params.lis_topic, Odometry, self.odom_cb)

        # pose service client
        rospy.wait_for_service(params.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(params.pose_srv_name, SharePoses2)

        # execute goal
        self.exec_cb()

        # # tensor force
        # self.tensor_force()

    def exec_cb(self):
        self.go_to_goal()
        self.is_reached = True
        return

    def go_to_goal(self):
        while (not self.ap.reached) and (not rospy.is_shutdown()):

            # record path
            self.rec.path_x.append(round(self.pose.x, 3))
            self.rec.path_y.append(round(self.pose.y, 3))

            # get all robots' data
            req_poses = SharePoses2Request()
            req_poses.id = self.rid
            req_poses.update = False
            req_poses.stopped = False
            all_robots_data: SharePoses2Response = self.pose_client(req_poses)

            # command ************************************
            self.ap.planner_move(self.pose, all_robots_data)
            self.v = self.ap.v
            self.w = self.ap.w
            self.rec.v_lin.append(self.v)
            self.rec.v_ang.append(self.w)

            # vizualize
            self.vs.robot_poly([[], self.ap.mp_bound], self.ns)
            self.vs.draw_robot_circles(self.ap.multi_robots_vis, self.ns)
            self.vizualize_force([self.ap.f_r, self.ap.f_theta], False)

            # publish cmd
            move_cmd = Twist()
            move_cmd.linear.x = self.v
            move_cmd.angular.z = self.w
            self.cmd_vel_pub.publish(move_cmd)

            # check stop
            if self.ap.stopped:
                req = SharePoses2Request()
                req.id = self.rid
                req.stopped = True
                self.pose_client(req)

            # log data
            self.log_motion(self.ap.f_r, self.ap.f_theta)
            self.rate.sleep()

            #
            if self.ap.reached:
                rospy.loginfo(f"[planner_ros, {self.ns}]: robot reached goal.")

        # reached
        req = SharePoses2Request()
        req.id = self.rid
        req.reached = True
        self.pose_client(req)
        self.stop()
        self.stop()
        rospy.loginfo(f"[planner_ros, {self.ns}]: go_to_goal finished!")

    def check_topic(self, lis_topic):
        topic_msg: Odometry = None
        rospy.loginfo(f"[planner_ros, {self.ns}]: checking topic ...")
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(lis_topic, Odometry, timeout=3.0)
                rospy.logdebug(f"[planner_ros, {self.ns}]: current topic is ready!")
            except rospy.ROSInterruptException as e:
                rospy.logerr(f"[planner_ros]: ROS node was interrupted: {e}")
            except rospy.ROSException as e:
                rospy.logerr(f"[planner_ros, {self.ns}]: Timeout while waiting for message: {e}")
                rospy.loginfo(f"[planner_ros, {self.ns}]: current topic is not ready yet, retrying ...")
        position = topic_msg.pose.pose.position
        quaternion = topic_msg.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.theta = orientation[2]

    def odom_cb(self, odom: Odometry):
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.theta = orientation[2]

    def stop(self):
        t = 0
        while t < 5:
            self.cmd_vel_pub.publish(Twist())
            self.rate.sleep()
            t += 1

    def log_motion(self, f_r, f_theta):
        n = 1
        if self.rid == n:
            v = round(self.v, 2)
            w = round(self.w, 2)
            fr = round(f_r, 2)
            ft = round(f_theta, 2)
            # rospy.loginfo(f"[planner_ros, {self.ns}]: {self.ap.stop_flag_multi}")
            rospy.loginfo(f"[planner_ros, {self.ns}]: f_r: {fr}, f_theta: {ft}")
            rospy.loginfo(f"[planner_ros, {self.ns}]: v: {v}, w: {w}")
            rospy.loginfo(" --------------------------------------------------- ")

    def shutdown_hook(self):
        rospy.loginfo(f"[planner_ros, {self.ns}]: shutting ...")
        self.stop()

    def vizualize_force(self, nr_force, ip=True):
        theta = np.arctan2(nr_force[1], nr_force[0])
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        theta = self.pose.theta + theta
        if ip:
            self.vs.arrow(self.pose.x, self.pose.y, theta)
        else:
            self.vs.arrow(self.pose.x, self.pose.y, theta, self.ns)
