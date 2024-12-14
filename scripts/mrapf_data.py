""" test informaiton, result file paths """
from typing import List, Dict
import os
import rospkg


class TestInfo:
    v: int
    n_robots: int
    res_file_p: str
    okayy: str

    def __init__(self, v: int = 1, n_robots: int = 2):
        self.v = v
        self.n_robots = n_robots
        self.res_file_p = ""
        self.create_paths()  # create result files paths

    def create_paths(self):
        # names
        folder_name = "T" + str(self.n_robots) + "_V" + str(self.v)
        file_name = "res_" + str(self.n_robots) + "_v" + str(self.v) + ".json"
        # Create directory
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        result_folder = os.path.join(pkg_path, "Results-APF/Tests")
        result_path = os.path.join(result_folder, folder_name)
        if not os.path.exists(result_path):
            os.makedirs(result_path)
        self.res_file_p = result_path+'/'  # os.path.join(result_path, file_name)


class TimeData:
    start: float
    end: float
    dur: float

    def __init__(self, start: float, end: float):
        self.start = start
        self.end = end
        self.dur = 0
        self.cal_dur()

    def cal_dur(self):
        self.dur = round(self.end - self.start, 3)


class PlannerData:
    time: TimeData
    x: List[float]
    y: List[float]
    xy: Dict[str, List[float]]
    f_tt: List[float]
    f_or: List[float]
    f_ot: List[float]
    phiis: List[float]

    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y
        self.xy = {"x": x, "y": y}
        self.time = None
        #
        self.f_tr = None
        self.f_tt = None
        self.f_or = None
        self.f_ot = None
        self.phiis = None

    def set_time(self, start: float, end: float):
        self.time = TimeData(start, end)


class AllPlannersData:
    n: int
    planners_data: List[PlannerData]
    all_x: List[List[float]]
    all_y: List[List[float]]
    all_xy: Dict[str, Dict[str, List[int]]]
    all_times: Dict[str, float]

    def __init__(self):
        self.n = 0
        self.all_x = []
        self.all_y = []
        self.all_xy = dict()
        self.all_times = dict()
        self.planners_data = []

    def add_data(self, p_data: PlannerData):
        self.planners_data.append(p_data)
        self.all_x.append(p_data.x)
        self.all_y.append(p_data.y)
        self.all_xy[str(self.n)] = p_data.xy
        self.all_times[str(self.n)] = p_data.time.dur
        self.n += 1
