#! /usr/bin/env python

import tf
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion

class Initialize(object):

    def __init__(self, model, path_unit=0.5):

        self.model = model
        self.obsts = {'x': [], 'y': '', 'count': 0}
        self.path_unit = path_unit
        self.robots_data = []
        self.robots_count = 1
        self.id = []
        
        # robots_initial
        self.multi_json()

    def multi_json(self):
        # obstacles
        x = [ox*self.path_unit for ox in self.model.obst.x]
        y = [oy*self.path_unit for oy in self.model.obst.y]
        self.obsts = {'x': x, 'y': y, 'count': self.model.obst.count}

        # robots
        robots_count = self.model.robots_i.robot_count
        self.robots_count = robots_count
        xx = [x*self.path_unit for x in self.model.robots_i.xs]
        yy = [y*self.path_unit for y in self.model.robots_i.ys]
        YY = [0 for t in self.model.robots_i.xs]
        self.id = self.model.robots_i.ids
        self.robots_initial = self.robots_initialize(robots_count, xx, yy, YY)

    def robots_initialize(self, robots_count, x, y, yaw):
        robots_initial = [Pose() for n in range(robots_count)]
        for rd in range(robots_count):
            theta = tf.transformations.quaternion_from_euler(0, 0, yaw[rd])
            robots_initial[rd].position = Point(x[rd], y[rd], 0)
            orient = Quaternion(*theta)
            robots_initial[rd].orientation = orient
        return robots_initial


# --------------------------------------------------------------------------------------------

def Spawning(model):
    # settings
    path_unit = 0.5

    # get data
    init_obj = Initialize(model, path_unit)
    robots_initial = init_obj.robots_initial
    robots_count = init_obj.robots_count
    obstacles = init_obj.obsts

    # ros
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('init_sim')

    # spawn_urdf_model service
    print("Waiting for gazebo spawn_urdf_model services for robots...")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawn_robots_servie = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    # robot file
    with open(pkg_path+'/models/TB3_model_simple.urdf', 'r') as file:
        robot_file = file.read()

    # spawn robots
    name = 'robot'
    reference_frame = 'map'

    for rd in range(robots_count):
        sm = SpawnModelRequest()
        id = init_obj.id[rd]
        sm.model_name = name+str(id)
        sm.model_xml = robot_file
        sm.robot_namespace = '/r'+str(id)
        sm.initial_pose = robots_initial[rd]
        sm.reference_frame = reference_frame
        # rospy.set_param('tf_prefix', '/r'+str(id))
        spawn_robots_servie(sm)
        rospy.sleep(0.2)

# --------------------------------------------------------------------------------------------

# # spawn obstacles
# # spawn_sdf_model sercice
# print("Waiting for gazebo spawn_sdf_model services for obstacles...")
# rospy.wait_for_service("gazebo/spawn_sdf_model")
# spawn_obst = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

# # model file
# with open(pkg_path+'/models/cylinder1/model2.sdf', 'r') as file2:
#     model_file = file2.read()

# # spawn models
# name = 'cylinder'
# model_namespace = ''
# reference_frame = 'world'

# for i in range(obstacles['count']):
#     pose = Point(x=obstacles['x'][i], y=obstacles['y'][i], z=0)
#     init_pose = Pose(pose, Quaternion())
#     model_name = name+str(i)
#     spawn_obst(model_name, model_file, model_namespace,
#                init_pose, reference_frame)
#     rospy.sleep(0.2)

