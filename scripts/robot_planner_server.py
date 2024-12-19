#! /usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from planner_ros import PlannerROS
from parameters import Params
from mrapf_classes import PlannerRobot, PlannerData
from apf.srv import SharePoses2, SharePoses2Request
from apf.msg import ApfAction, ApfResult, ApfFeedback
# planner_4_3 - planner_1


class RobotPlanner:
    p_data: PlannerData

    def __init__(self, model, robot: PlannerRobot, action_params: Params):

        #
        self.mp = 0  # pre
        self.robot = robot  # attributes
        self.model = model  # shared data
        self.p_data = None

        # setting - parameters
        self.action_params = action_params
        self.name_s = action_params.name_space

        # shutdown hook
        rospy.on_shutdown(self.shutdown)

        # pose service client
        rospy.wait_for_service(action_params.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(action_params.pose_srv_name, SharePoses2)

        # action: /r#/apf_action ===============================================
        self.result = ApfResult()
        self.feedback = ApfFeedback()
        self.ac_name = action_params.ac_name
        self._as = actionlib.SimpleActionServer(self.ac_name, ApfAction, self.goal_callback, False)
        self._as.start()
        rospy.loginfo(f"[RobotPlanner, {self.name_s}]: Robot Action Server [{self.ac_name}] has started.")

    def goal_callback(self, goal):

        rospy.loginfo(f"[RobotPlanner, {self.name_s}]: Robot Action Server [{self.ac_name}] is called.")

        # goal received
        self.robot.xt = goal.xt * self.action_params.path_unit
        self.robot.yt = goal.yt * self.action_params.path_unit

        # get robots initial odom - start state
        self.get_odom()

        # update target coordinates in pose service
        req = SharePoses2Request()
        req.id = self.robot.id
        req.xt = self.robot.xt
        req.yt = self.robot.yt
        req.update = True
        self.pose_client(req)

        # self.time: [start, end, duration]
        # start time
        time_start = rospy.get_time()

        # motion planning   # ==================================================
        planner = PlannerROS(self.model, self.robot, self.action_params)
        success = planner.is_reached

        # time
        time_end = rospy.get_time()

        # result
        if success:
            self.result.result = True
            self.result.path_x = planner.rec.path_x
            self.result.path_y = planner.rec.path_y
            rospy.loginfo(f"[RobotPlanner, {self.name_s}]: Succeeded!")
            self._as.set_succeeded(self.result)
            self.p_data = PlannerData(planner.rec.path_x, planner.rec.path_y)
            self.p_data.set_time(time_start, time_end)
        else:
            rospy.loginfo(f'Failed {self.ac_name}')
            self._as.set_aborted(self.result)

    def get_odom(self):
        topic_msg: Odometry = None
        rospy.loginfo(self.ac_name + ", getting topic ...")
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(self.action_params.lis_topic, Odometry, timeout=3.0)
                rospy.logdebug(self.ac_name + ", current topic is ready.")
            except rospy.ROSInterruptException as e:
                rospy.logerr(f"ROS node was interrupted: {e}")
            except rospy.ROSException as e:
                rospy.logerr(f"Timeout while waiting for message: {e}")
                rospy.loginfo(f"{self.ac_name}, current topic is not ready yet, retrying ...")
        odom = topic_msg
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        orientation = orientation[2]
        position = odom.pose.pose.position
        self.robot.xs = position.x
        self.robot.ys = position.y
        self.robot.heading = round(orientation, 2)

    def shutdown(self):
        rospy.loginfo(f"[RobotPlanner, {self.name_s}]: shutting down ... ")
        # rospy.sleep(1)
