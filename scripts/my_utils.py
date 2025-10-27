import numpy as np


def cal_distance(x1, y1, x2, y2) -> float:
    return np.sqrt((x1-x2)**2+(y1-y2)**2)


def mod_angle(theta) -> float:
    if theta < 0:
        theta = theta + 2 * np.pi
    elif theta > 2 * np.pi:
        theta = theta - 2 * np.pi
    return theta


def cal_angle_diff(a1, a2) -> float:
    ad = a1 - a2
    ad = np.arctan2(np.sin(ad), np.cos(ad))
    return ad
