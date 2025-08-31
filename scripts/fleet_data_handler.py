#! /usr/bin/env python

""" Fleet Data Handler for MRAPF """

from typing import Dict, Tuple, List
import rospy
import tf2_ros
import tf_conversions
from parameters import Params
from geometry_msgs.msg import Pose2D
from apf.msg import RobotData, FleetData
from apf.srv import SendRobotUpdate, SendRobotUpdateRequest, SendRobotUpdateResponse


class FleetDataHandler:
    nr: int
    rids: List[int]
    sns: Dict[int, str]
    xy: Dict[int, Tuple[float, float]]
    xyt: Dict[int, Tuple[float, float]]
    poses: Dict[int, Pose2D]
    fleet_data: Dict[int, RobotData]
    frames: Dict[int, str]

    def __init__(self, params: Params):

        # data
        self.nr = 0
        self.rids = []
        self.sns = {}
        self.xy = {}
        self.xyt = {}
        self.poses = {}
        self.fleet_data = {}
        self.frames = {}

        # settings
        self.global_frame = params.global_frame
        self.local_frame = params.local_frame

        # tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # service
        self.srv = rospy.Service(params.sru_srv_name, SendRobotUpdate, self.sru_cb)

        # publisher
        self.pub = rospy.Publisher(params.fleet_data_topic, FleetData, queue_size=10)

        # # update and pubish data
        # self.update_all()

    def add_robot(self, rid: int, sns: str = ""):
        self.nr += 1
        self.rids.append(rid)
        self.sns[rid] = sns
        self.xy[rid] = (0, 0)
        self.xyt[rid] = (0, 0)
        self.poses[rid] = Pose2D()
        self.fleet_data[rid] = RobotData()
        self.fleet_data[rid].rid = rid
        self.frames[rid] = sns + self.local_frame

    def update_goal(self, rid: int, xt: float, yt: float):
        self.xyt[rid] = (xt, yt)

    def sru_cb(self, req: SendRobotUpdateRequest) -> SendRobotUpdateResponse:
        rid = req.rid
        self.fleet_data[rid].stopped = req.stopped
        self.fleet_data[rid].reached = req.reached
        resp = SendRobotUpdateResponse()
        resp.success = True
        return resp

    def update_all(self):
        fd = FleetData()
        if self.get_all_tf():
            self.update_priority()
            fd.success = True
            fd.nr = self.nr
            fd.fdata = list(self.fleet_data.values())
            self.pub.publish(fd)
        else:
            fd.success = False
            self.pub.publish(fd)
            rospy.logwarn("[FleetDataHandler]: Failed to get all TFs")

    def get_all_tf(self) -> bool:
        for rid in self.rids:
            if self.fleet_data[rid].reached:
                continue
            if not self.get_tf(rid, self.frames[rid]):
                return False
        return True

    def get_tf(self, rid: int, frame: str) -> bool:
        try:
            trans = self.tf_buffer.lookup_transform(self.global_frame, frame, rospy.Time(0), rospy.Duration(0.2))
            translation = trans.transform.translation
            rotation = trans.transform.rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[FleetDataHandler]: TF Exception for robot with frame {frame}: {e}")
            return False
        except Exception as e:  # pylint: disable-all
            rospy.logwarn(f"[FleetDataHandler]: Exception for robot with frame {frame}: {e}")
            return False
        # Extract the x, y, and yaw from the transformation
        x = translation.x
        y = translation.y
        euler = tf_conversions.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        theta = euler[2]
        #
        self.xy[rid] = (x, y)
        #
        self.fleet_data[rid].x = x
        self.fleet_data[rid].y = y
        self.fleet_data[rid].h = theta
        return True

    def update_priority(self):
        dists = []
        max_dist = 0.001
        for rid in self.rids:
            x_t, y_t = self.xyt[rid]
            x_r, y_r = self.fleet_data[rid].x, self.fleet_data[rid].y
            d = ((x_t - x_r) ** 2 + (y_t - y_r) ** 2) ** 0.5
            dists.append(d)
            if d > max_dist:
                max_dist = d
        dists = [d/max_dist for d in dists]
        for i, rid in enumerate(self.rids):
            self.fleet_data[rid].priority = dists[i]

    # -------------------------------------------------------------------------

    def get_robot_pose(self, rid: int) -> Pose2D:
        return self.poses[rid]

    def get_fleet_data(self) -> FleetData:
        fd = FleetData()
        fd.success = True
        fd.nr = self.nr
        fd.fdata = list(self.fleet_data.values())
        return fd

    def set_robot_pose(self, rid: int, x: float, y: float, h: float):
        self.xy[rid] = (x, y)
        self.fleet_data[rid].x = x
        self.fleet_data[rid].y = y
        self.fleet_data[rid].h = h
        self.poses[rid] = Pose2D(x, y, h)

    def set_robot_data(self, rid: int, stopped: bool, reached: bool):
        self.fleet_data[rid].stopped = stopped
        self.fleet_data[rid].reached = reached
        self.fleet_data[rid].priority = 0.0

    # def get_all_data(self) -> Dict[int, RobotData]:
    #     return self.fleet_data
