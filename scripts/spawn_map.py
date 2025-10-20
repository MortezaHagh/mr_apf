""" spawn robots (and obstacles) in the world based on model data """
#! /usr/bin/env python

from typing import List
import rospkg  # type: ignore
import rospy
import tf
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from create_model import MRSModel


class SimData(object):

    def __init__(self, model: MRSModel):
        self.model: MRSModel = model
        self.rids: List[int] = []
        self.all_obsts: List[Pose] = []
        self.all_robots: List[Pose] = []

        # create spawning data
        self.create_spawn_data()

    def create_spawn_data(self):
        # obstacles
        for obst in self.model.obstacles:
            p = Pose()
            p.position.z = 0
            p.position.x = obst.x
            p.position.y = obst.y
            p.orientation.w = 1.0
            self.all_obsts.append(p)

        # robots
        self.all_robots: List[Pose] = []
        for robot in self.model.robots:
            self.rids.append(robot.rid)
            p = Pose()
            p.position.z = 0.0
            p.position.x = robot.xs
            p.position.y = robot.ys
            theta = tf.transformations.quaternion_from_euler(0, 0, robot.heading)
            p.orientation = Quaternion(*theta)
            self.all_robots.append(p)


def spawning(model):

    # get data
    sim_data = SimData(model)

    # spawn_urdf_model service
    rospy.loginfo("[spawning]: Waiting for gazebo spawn_urdf_model services for robots...")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn_robots_servie = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    # robot model file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('apf')
    with open(pkg_path+'/models/TB3_model_simple.urdf', 'r') as file:
        robot_file = file.read()

    # spawn robots
    name = 'robot'
    reference_frame = 'map'

    for i, pose in enumerate(sim_data.all_robots):
        rospy.loginfo("[spawning]: spawn robot: " + str(sim_data.rids[i]))
        sm = SpawnModelRequest()
        id = sim_data.rids[i]
        sm.model_name = name+str(id)
        sm.model_xml = robot_file
        sm.robot_namespace = '/r'+str(id)
        sm.initial_pose = pose
        sm.reference_frame = reference_frame
        rospy.set_param('tf_prefix', 'r'+str(id))
        spawn_robots_servie(sm)
        rospy.sleep(0.2)

    rospy.loginfo("[spawning]: spawning done!")

    # --------------------------------------------------------------------------

    # # # spawn obstacles
    # # spawn_sdf_model sercice
    # rospy.loginfo("[spawning]: Waiting for gazebo spawn_sdf_model services for obstacles...")
    # rospy.wait_for_service("gazebo/spawn_sdf_model")
    # spawn_obst = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    # # model file
    # with open(pkg_path+'/models/cylinder1/model2.sdf', 'r') as file2:
    #     model_file = file2.read()

    # # spawn models
    # name = 'cylinder'
    # model_namespace = ''
    # reference_frame = 'world'

    # for i, pose in enumerate(sim_data.all_obsts):
    #     model_name = name+str(i)
    #     spawn_obst(model_name, model_file, model_namespace, pose, reference_frame)
    #     rospy.sleep(0.2)
