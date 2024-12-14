""" spawn robots (and obstacles) in the world based on model data """
#! /usr/bin/env python

import tf
import rospy
import rospkg
from typing import List
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from create_model import MRSModel


class Initialize(object):
    model: MRSModel
    path_unit: float
    ids: List[int]
    all_robots: List[Pose]
    all_obsts: List[Pose]

    def __init__(self, model: MRSModel, path_unit: float = 1.0):
        self.model = model
        self.path_unit = path_unit
        self.ids = []
        self.all_obsts = []
        self.all_robots = []

        # create spawning data
        self.create_spawn_data()

    def create_spawn_data(self):
        # obstacles
        for i in range(self.model.obst.count):
            p = Pose()
            p.position.z = 0
            p.position.x = self.model.obst.x[i]*self.path_unit
            p.position.y = self.model.obst.x[i]*self.path_unit
            p.orientation.w = 1.0
            self.all_obsts.append(p)

        # robots
        self.all_robots: List[Pose] = []
        for robot in self.model.robots:
            self.ids.append(robot.id)
            p = Pose()
            p.position.z = 0.0
            p.position.x = robot.xs*self.path_unit
            p.position.y = robot.ys*self.path_unit
            theta = tf.transformations.quaternion_from_euler(0, 0, robot.heading)
            p.orientation = Quaternion(*theta)
            self.all_robots.append(p)


def Spawning(model, path_unit=1.0):

    # get data
    init_obj = Initialize(model, path_unit)

    # spawn_urdf_model service
    print("Waiting for gazebo spawn_urdf_model services for robots...")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn_robots_servie = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    # robot file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('apf')
    with open(pkg_path+'/Models/TB3_model_simple.urdf', 'r') as file:
        robot_file = file.read()

    # spawn robots
    name = 'robot'
    reference_frame = 'map'

    for i, pose in enumerate(init_obj.all_robots):
        rospy.loginfo("spawning robot: " + str(init_obj.ids[i]))
        sm = SpawnModelRequest()
        id = init_obj.ids[i]
        sm.model_name = name+str(id)
        sm.model_xml = robot_file
        sm.robot_namespace = '/r'+str(id)
        sm.initial_pose = pose
        sm.reference_frame = reference_frame
        rospy.set_param('tf_prefix', 'r'+str(id))
        spawn_robots_servie(sm)
        rospy.sleep(0.2)

    print("spawning done!")

    # --------------------------------------------------------------------------

    # # # spawn obstacles
    # # spawn_sdf_model sercice
    # print("Waiting for gazebo spawn_sdf_model services for obstacles...")
    # rospy.wait_for_service("gazebo/spawn_sdf_model")
    # spawn_obst = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    # # model file
    # with open(pkg_path+'/Models/cylinder1/model2.sdf', 'r') as file2:
    #     model_file = file2.read()

    # # spawn models
    # name = 'cylinder'
    # model_namespace = ''
    # reference_frame = 'world'

    # for i, pose in enumerate(init_obj.all_obsts):
    #     model_name = name+str(i)
    #     spawn_obst(model_name, model_file, model_namespace, pose, reference_frame)
    #     rospy.sleep(0.2)
