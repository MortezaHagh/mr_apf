def create_model_set():
    params = Params()
    for i in range(1, 13):
        model = MRSModel(n_robots=i)
        plot_model(model, params)

        # save fig
        ind = str(model.map_ind)
        o_ind = str(model.obst_count_orig)
        no = 'o'+o_ind+'_map'+ind
        map_name = 'maps/'+no  # + '_e'
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        filename = os.path.join(pkg_path, map_name)
        plt.savefig(filename+'.svg', format="svg", dpi=1000)
        plt.savefig(filename+'.png', format="png", dpi=500)

    # plt.show()