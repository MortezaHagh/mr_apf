import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def plot_model(model, settings):
    # setting
    obs_r = settings.obst_r
    obst_prec_d = settings.obst_prec_d

    fig, ax = plt.subplots(1, 1)
    fig.set_size_inches(8, 8)
    # ax.set_title('MRPP')
    ax.axis('equal')
    # ax.axis("off")
    # ax.grid('on')
    ax.axis([model.map.x_min-1, model.map.x_max+1, model.map.y_min-1, model.map.y_max+1])

    # robots start and target nodes
    colors = plt.cm.get_cmap('rainbow', len(model.robots))
    for i, robot in enumerate(model.robots):
        ax.plot(robot.xs, robot.ys, marker='s', markersize=10,
                markeredgecolor=colors(i), markerfacecolor=colors(i))
        ax.plot(robot.xt, robot.yt, marker='p', markersize=10,
                markeredgecolor=colors(i), markerfacecolor=colors(i))
        # text
        ax.text(robot.xs, robot.ys, str(i+1), {'fontsize': 10, 'fontweight': 'normal',
                'horizontalalignment': 'center', 'fontname': 'serif'})
        ax.text(robot.xt, robot.yt, str(i+1), {'fontsize': 10, 'fontweight': 'normal',
                'horizontalalignment': 'center', 'fontname': 'serif'})

    # # Obstacles
    thetas = np.linspace(0, np.pi*2, 20)
    ax.plot(model.obst.x, model.obst.y, 'o',  markersize=5,
            markeredgecolor='k', markerfacecolor='k')
    for i in range(model.obst.count):
        xor = [model.obst.x[i]+obst_prec_d*np.cos(t) for t in thetas]
        yor = [model.obst.y[i]+obst_prec_d*np.sin(t) for t in thetas]
        xdng = [model.obst.x[i]+obs_r*np.cos(t) for t in thetas]
        ydng = [model.obst.y[i]+obs_r*np.sin(t) for t in thetas]
        ax.plot(xor, yor, '--k')
        ax.plot(xdng, ydng, 'r')

    # Walls
    lx = model.map.x_max-model.map.x_min + 1
    ly = model.map.y_max-model.map.y_min + 1
    rect = patches.Rectangle((model.map.x_min-0.5, model.map.y_min-0.5),
                             lx, ly, linewidth=2, edgecolor='k', facecolor='none')
    ax.add_patch(rect)

    return fig, ax


def plot_forces(ac, dir_force):
    fig, (ax, ax2) = plt.subplots(2, 1)
    line1, = ax.plot(ac.mp.force_tr, '-b', label="target r")
    line2, = ax.plot(ac.mp.force_tt, '-g', label="target t")
    line3, = ax.plot(ac.mp.force_or, '-r', label="object r")
    line4, = ax.plot(ac.mp.force_ot, '-k', label="object t")
    line5, = ax2.plot(ac.mp.phiis, '--', label="phi")

    ax.legend(handles=[line1, line2, line3, line4])
    ax2.legend(handles=[line5])
    fig.savefig(dir_force+".svg", format="svg")
    # plt.show()


if __name__ == '__main__':
    from create_model import CreateModel
    model = CreateModel(map_id=4)
    plot_model(model)
    plt.show()
