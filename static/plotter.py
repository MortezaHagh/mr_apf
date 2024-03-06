import numpy as np
from parameters import Params
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def plot_model(model, settings):

    # settings
    robot_r = settings.robot_r
    danger_r = settings.danger_r
    obs_r = settings.obs_effect_r

    fig, ax = plt.subplots(1, 1)
    # ax.set_title('MRPP')
    # ax.axis("off")
    ax.axis('equal')
    ax.grid('on')
    ax.axis([model.map.x_min-1, model.map.x_max+1,
            model.map.y_min-1, model.map.y_max+1])

    # robots start and target nodes
    colors = plt.cm.get_cmap('rainbow', model.robot_count)
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
        xor = [model.obst.x[i]+obs_r*np.cos(t) for t in thetas]
        yor = [model.obst.y[i]+obs_r*np.sin(t) for t in thetas]
        xdng = [model.obst.x[i]+danger_r*np.cos(t) for t in thetas]
        ydng = [model.obst.y[i]+danger_r*np.sin(t) for t in thetas]
        ax.plot(xor, yor, '--k')
        ax.plot(xdng, ydng, 'r')

    # Walls
    lx = model.map.x_max-model.map.x_min + 1
    ly = model.map.y_max-model.map.y_min + 1
    rect = patches.Rectangle((model.map.x_min-0.5, model.map.y_min-0.5),
                             lx, ly, linewidth=2, edgecolor='k', facecolor='none')
    ax.add_patch(rect)

    return fig, ax


if __name__ == '__main__':
    params = Params()
    from create_model import CreateModel
    model = CreateModel(map_id=1)
    plot_model(model, params)
    plt.show()
