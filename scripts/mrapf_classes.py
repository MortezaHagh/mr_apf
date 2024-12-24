""" test informaiton, result file paths """
from typing import List, Dict
import os
import rospkg
from parameters import Params


class TestInfo:
    name: str
    sim: str
    nr: int
    method: int
    res_file_p: str

    def __init__(self, params: Params, name: str = ""):
        if name != "":
            name = name + "_"
        self.name = name
        self.nr = params.nr
        self.sim = params.sim
        self.method = params.method
        self.res_file_p = ""
        self.create_paths()  # create result files paths

    def create_paths(self):
        # names
        folder_name = self.name + "T" + str(self.nr) + "_M" + str(self.method) + "_" + self.sim
        # Create directory
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        result_folder = os.path.join(pkg_path, "Results-APF/Tests")
        result_path = os.path.join(result_folder, folder_name)
        if not os.path.exists(result_path):
            os.makedirs(result_path)
        self.res_file_p = result_path+'/'


class PlannerData:
    steps: int
    start_t: float
    end_t: float
    dur_t: float
    x: List[float]
    y: List[float]
    xy: Dict[str, List[float]]
    f_tt: List[float]
    f_tr: List[float]
    f_or: List[float]
    f_ot: List[float]
    phis: List[float]

    def __init__(self):
        self.start_t = None
        self.end_t = None
        self.dur_t = None
        self.steps = 0
        # v
        self.v = []
        self.w = []
        # xy
        self.x = []
        self.y = []
        self.xy = {"x": [], "y": []}
        # f
        self.f_tr = []
        self.f_tt = []
        self.f_or = []
        self.f_ot = []
        self.phis = []

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
    n: int
    all_steps: List[int]
    all_x: List[List[float]]
    all_y: List[List[float]]
    all_times: Dict[str, float]
    planners_data: List[PlannerData]
    all_xy: Dict[str, Dict[str, List[int]]]

    def __init__(self):
        self.n = 0
        self.all_x = []
        self.all_y = []
        self.all_xy = {}
        self.all_steps = []
        self.all_times = {}
        self.planners_data = []

    def add_data(self, p_data: PlannerData):
        self.planners_data.append(p_data)
        self.all_x.append(p_data.x)
        self.all_y.append(p_data.y)
        self.all_xy[str(self.n)] = p_data.xy
        self.all_steps.append(p_data.steps)
        self.all_times[str(self.n)] = p_data.dur_t
        self.n += 1


class ApfRobot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 1
        self.d = 0
        self.H = 0
        self.h_rR = 0
        self.theta_rR = 0
        self.r_prec = 0
        self.r_half = 0
        self.r_start = 0
        self.p = False
        self.big = False
        self.stopped = False
        self.reached = False
