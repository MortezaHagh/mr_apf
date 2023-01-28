import numpy as np
import matplotlib.pyplot as plt


def plot_forces(ac):
    fig, ax = plt.subplots(1, 1)
    line1, = ax.plot(ac.mp.force_tr, '-b', label = "target r")
    line2, = ax.plot(ac.mp.force_tt, '-g', label = "target t")
    line3, = ax.plot(ac.mp.force_or, '-r', label = "object r")
    line4, = ax.plot(ac.mp.force_ot, '-k', label = "object t")
    line5, = ax.plot(ac.mp.phiis, '-r', label = "phi")

    ax.legend(handles=[line1, line2, line3, line4, line5])
    # plt.show()

# self.force_tr = []
#         self.force_tt = []
#         self.force_or = []
#         self.force_ot = []
#         self.phiis = []