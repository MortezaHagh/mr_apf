#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from parameters import Params
from mrapf_classes import PRobot
from create_model import MRSModel
from robot_planner_base import RobotPlanner
from apf.msg import FleetData
from apf.srv import SendRobotUpdate, SendRobotUpdateRequest


class PlannerRT(RobotPlanner):
    def __init__(self, model: MRSModel, robot: PRobot, params: Params):
        RobotPlanner.__init__(self, model, robot, params)

        # listener
        self.data_received = False
        rospy.Subscriber(params.fleet_data_topic, FleetData, self.fleet_data_cb, queue_size=2)

        # Send Robot Update - target
        rospy.wait_for_service(params.sru_srv_name)
        self.robot_update_client = rospy.ServiceProxy(params.sru_srv_name, SendRobotUpdate)
        self.ruc_req = SendRobotUpdateRequest()
        self.ruc_req.rid = self.rid
        self.ruc_req.xt = robot.xt
        self.ruc_req.yt = robot.yt
        self.ruc_req.stopped = False
        self.ruc_req.reached = False

    def go_to_goal(self):
        while (not self.ap.reached) and (not rospy.is_shutdown()):
            if not self.data_received:
                rospy.loginfo(f"[planner_ros, {self.ns}]: waiting for fleet data...")
                self.rate.sleep()
                continue

            # update robot's data
            self.ruc_req.stopped = False

            # command *********************************************************
            self.ap.planner_move(self.pose, self.fleet_data)
            self.v = self.ap.v
            self.w = self.ap.w
            self.pd.v.append(self.v)
            self.pd.w.append(self.w)

            # record path
            self.pd.x.append(round(self.pose.x, 3))
            self.pd.y.append(round(self.pose.y, 3))

            # vizualize
            if self.do_viz:
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
                self.ruc_req.stopped = True
                self.robot_update_client(self.ruc_req)

            # log data
            self.log_motion(self.ap.f_r, self.ap.f_theta)
            self.rate.sleep()

            #
            if self.ap.reached:
                rospy.loginfo(f"[planner_ros, {self.ns}]: robot reached goal.")

        # reached
        self.ruc_req.stopped = True
        self.ruc_req.reached = True
        self.robot_update_client(self.ruc_req)
        self.stop()
        self.stop()
        rospy.loginfo(f"[planner_ros, {self.ns}]: go_to_goal finished!")

    def fleet_data_cb(self, msg: FleetData):
        if msg.success:
            self.data_received = True
        else:
            self.data_received = False
        self.fleet_data = msg
