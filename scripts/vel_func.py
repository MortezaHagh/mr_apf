#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

class FuncV:
    def __init__(self):

        #
        self.v_max = 0.2

        # settings
        self.zeta = 1                           # init_params.zeta
        self.fix_f = 4
        self.fix_f2 = 10
        self.robot_r = 0.22                      # init_params.robot_r
        self.obst_r = 0.15
        self.prec_d = 0.2
        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_start_d = self.obst_prec_d*2
        self.obst_z = 4*self.fix_f*self.obst_prec_d**4

        fig, (self.ax, self.ax2) = plt.subplots(2, 1)
        self.ax.set_title('V - F')
        # self.ax.axis('equal')
        self.ax.grid('on')

        self.ax2.set_title("F")
        # self.ax.axis('equal')
        self.ax2.grid('on')


    
    def vFunc(self, f_r):
        # v = 1 * self.v_max * ((f_r / self.fix_f)**2 + (f_r / self.fix_f) / 4) + 0.01
        v = 1 * self.v_max *  ((f_r / self.fix_f) / 1) + 0.01
        return v
    
    def plot_v(self):
        FR = np.linspace(0, 4, 100)
        V = []
        for f in FR:
            v = self.vFunc(f)
            V.append(v)

        self.ax.plot(FR, V)

    def fFunc(self, d):
        f = ((self.obst_z * 1) * ((1 / d) - (1 / self.obst_start_d))**2) * (1 / d)**2
        return f
    
    def plot_F(self):
        D = np.linspace(0.8*self.obst_prec_d, 3*self.obst_prec_d, 100)
        F = []
        Ft = []
        for d_ro in D:
            f = self.fFunc(d_ro)
            F.append(f)

        f_start = self.fFunc(self.obst_start_d)
        f_prec = self.fFunc(self.obst_prec_d)

        self.ax2.plot(D, F, 'b')
        self.ax2.plot(self.obst_start_d, f_start, "og")
        self.ax2.plot(self.obst_prec_d, f_prec, "or")



if __name__=="__main__":
    fv = FuncV()
    fv.plot_v()
    fv.plot_F()
    plt.show()


