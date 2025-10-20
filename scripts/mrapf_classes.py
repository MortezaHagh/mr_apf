""" Classes TestInfo, PlannerData, AllPlannersData, ApfRobot """

from typing import List, Dict
import os
import rospkg
from parameters import Params


class TestInfo:

    def __init__(self, params: Params, name: str = ""):
        if name != "":
            name = name + "_"
        self.name: str = name
        self.map_id: int = params.map_id
        self.nr: int = params.nr
        self.simD: str = params.simD
        self.method: int = params.method
        self.results_folder: str = params.results_folder
        self.res_file_path: str = ""
        self.create_paths()  # create result files paths

    def create_paths(self):
        # names
        folder_name = self.name + "Map" + str(self.map_id) + "_M" + str(self.method) + \
            "_" + self.simD + "_N" + str(self.nr)
        # Create directory
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        base_path = os.path.join(pkg_path, self.results_folder)
        result_path = os.path.join(base_path, folder_name)
        if not os.path.exists(result_path):
            os.makedirs(result_path)
        self.res_file_path = result_path+'/'


class PlannerData:
    def __init__(self):
        self.success: bool = False
        self.steps: int = 0
        self.start_t: float = None
        self.end_t: float = None
        self.dur_t: float = None
        # v
        self.v: List[float] = []
        self.w: List[float] = []
        # xy
        self.x: List[float] = []
        self.y: List[float] = []
        self.xy: Dict[str, List[float]] = {"x": [], "y": []}
        # forces
        self.f_tt: List[float] = []
        self.f_tr: List[float] = []
        self.f_or: List[float] = []
        self.f_ot: List[float] = []
        self.phis: List[float] = []

    def set_success(self, success: bool):
        self.success = success

    def append_data(self, x, y, v, w):
        self.add_xy(x, y)
        self.add_vw(v, w)
        self.steps += 1

    def set_start_time(self, start: float):
        self.start_t = start

    def set_end_time(self, end: float):
        self.end_t = end

    def add_xy(self, x: float, y: float):
        self.x.append(round(x, 2))
        self.y.append(round(y, 2))

    def add_vw(self, v: float, w: float):
        self.v.append(round(v, 2))
        self.w.append(round(w, 2))

    def finalize(self):
        self.xy = {"x": self.x, "y": self.y}
        self.dur_t = round(self.end_t - self.start_t, 3)


class AllPlannersData:

    def __init__(self):
        self.n: int = 0
        self.n_reached: int = 0
        self.all_x: List[List[float]] = []
        self.all_y: List[List[float]] = []
        self.all_xy: Dict[str, Dict[str, List[int]]] = {}
        self.all_steps: List[int] = []
        self.all_durations: Dict[str, float] = {}
        self.planners_data: List[PlannerData] = []

    def add_data(self, p_data: PlannerData):
        self.planners_data.append(p_data)
        self.all_x.append(p_data.x)
        self.all_y.append(p_data.y)
        self.all_xy[str(self.n)] = p_data.xy
        self.all_steps.append(p_data.steps)
        self.all_durations[str(self.n)] = p_data.dur_t
        self.n += 1
        if p_data.success:
            self.n_reached += 1


class ApfRobot:
    def __init__(self, x: float = 0, y: float = 0, d: float = 0, H: float = 0):
        self.x = x
        self.y = y
        self.d = d
        self.H = H
        self.h_rR = 0
        self.theta_rR = 0
        self.z = 1
        self.r_prec = 0
        self.r_half = 0
        self.r_start = 0
        self.prior = False  # has higher priority
        self.priority = 0
        self.cluster = False
        self.stopped = False
        self.reached = False
