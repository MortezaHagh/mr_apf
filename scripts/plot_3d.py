

def plot_f_obstacle(self):
        
        x_step = 0.1
        y_step = 0.1
        x = np.linspace(self.model.map.x_min, self.model.map.x_max, x_step)
        y = np.linspace(self.model.map.y_min, self.model.map.y_max, y_step)
        X, Y = np.meshgrid(x, y, indexing='ij')
        Z = np.zeros(X.shape)

        for i in range(X.shape[0]):
                for j in range(X.shape[1]):
                    xx = X[i,j]
                    yy = Y[i,j]

                    for i in self.obs_ind_main:
                        dy = (self.obs_y[i] - yy)
                        dx = (self.obs_x[i] - xx)
                        d_ro = np.sqrt(dx**2 + dy**2)

                        f = ((self.obst_z * 1) * ((1 / d_ro) - (1 / self.obst_start_d))**2) * (1 / d_ro)**2
                        Z[i,j] = f
        
        fig, ax = plt.figure()
        # ax = plt.axes(projection='3d')
        ax.contour3D(X, Y, Z, 50, cmap='binary')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')