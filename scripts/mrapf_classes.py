""" test informaiton, result file paths """
from typing import List, Dict
import os
import rospkg
from apf.srv import SharePosesResponse


class TestInfo:
    v: int
    ns: str
    method: int
    n_robots: int
    res_file_p: str

    def __init__(self, v: int = 1, n_robots: int = 2, method: int = 2, ns: str = ""):
        self.v = v
        self.ns = ns
        self.method = method
        self.n_robots = n_robots
        self.res_file_p = ""
        self.create_paths()  # create result files paths

    def create_paths(self):
        # names
        folder_name = self.ns + "T" + str(self.n_robots) + "_V" + str(self.v) + "_M" + str(self.method)
        # Create directory
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        result_folder = os.path.join(pkg_path, "Results-APF/Tests")
        result_path = os.path.join(result_folder, folder_name)
        if not os.path.exists(result_path):
            os.makedirs(result_path)
        self.res_file_p = result_path+'/'


class PRobot:
    def __init__(self, xs=0, ys=0, rid=0, name="r", heading=0, xt=0, yt=0):
        self.rid = rid
        self.xs = xs
        self.ys = ys
        self.xt = xt
        self.yt = yt
        self.name = name
        self.heading = heading
        self.priority = rid


class PlannerData:
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

    def finalize(self):
        self.xy = {"x": self.x, "y": self.y}
        self.dur_t = round(self.end_t - self.start_t, 3)


class AllPlannersData:
    n: int
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
        self.all_times = {}
        self.planners_data = []

    def add_data(self, p_data: PlannerData):
        self.planners_data.append(p_data)
        self.all_x.append(p_data.x)
        self.all_y.append(p_data.y)
        self.all_xy[str(self.n)] = p_data.xy
        self.all_times[str(self.n)] = p_data.dur_t
        self.n += 1


class AllRobotsData:
    nr: int
    x: List[float]
    y: List[float]
    h: List[float]
    pr: List[int]
    reached: List[bool]
    stopped: List[bool]

    def __init__(self):
        self.nr = []
        self.x = []
        self.y = []
        self.h = []
        self.pr = []
        self.reached = []
        self.stopped = []

    def update_by_resp(self, resp: SharePosesResponse):
        self.nr = resp.nr
        self.x = resp.x
        self.y = resp.y
        self.h = resp.h
        self.pr = resp.pr
        self.reached = resp.reached
        self.stopped = resp.stopped

    def update(self):
        pass


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
