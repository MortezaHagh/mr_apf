import numpy as np
import matplotlib.pyplot as plt


def plot_forces(ac, dir_force):
    fig, (ax, ax2) = plt.subplots(2, 1)
    line1, = ax.plot(ac.mp.force_tr, '-b', label = "target r")
    line2, = ax.plot(ac.mp.force_tt, '-g', label = "target t")
    line3, = ax.plot(ac.mp.force_or, '-r', label = "object r")
    line4, = ax.plot(ac.mp.force_ot, '-k', label = "object t")
    line5, = ax2.plot(ac.mp.phiis, '--', label = "phi")

    ax.legend(handles=[line1, line2, line3, line4])
    ax2.legend(handles=[line5])
    fig.savefig(dir_force+".png", format="png")
    # plt.show()