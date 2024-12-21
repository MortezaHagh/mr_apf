#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from apf.srv import SharePoses2, SharePoses2Request
from parameters import Params
from create_model import MRSModel
from mrapf_classes import PRobot
from robot_planner import RobotPlanner


class PlannerROS(RobotPlanner):
    def __init__(self, model: MRSModel, robot: PRobot, params: Params):
        RobotPlanner.__init__(self, model, robot, params)

        # listener
        self.check_topic(params.lis_topic)
        rospy.Subscriber(params.lis_topic, Odometry, self.odom_cb)

        # pose service client
        rospy.wait_for_service(params.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(params.pose_srv_name, SharePoses2)

    def go_to_goal(self):
        while (not self.ap.reached) and (not rospy.is_shutdown()):

            # record path
            self.pd.x.append(round(self.pose.x, 3))
            self.pd.y.append(round(self.pose.y, 3))

            # get all robots' data
            req_poses = SharePoses2Request()
            req_poses.id = self.rid
            req_poses.update = False
            req_poses.stopped = False
            self.ard.update_by_resp(self.pose_client(req_poses))

            # command ************************************
            self.ap.planner_move(self.pose, self.ard)
            self.v = self.ap.v
            self.w = self.ap.w
            self.pd.v.append(self.v)
            self.pd.w.append(self.w)

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
