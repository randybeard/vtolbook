"""
landing trajectory with minimum acceleration / jerk
        2/17/22 - RWB
"""
import numpy as np
from math import ceil
from scipy.interpolate import BSpline
from scipy.linalg import norm
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def plotSpline(spl):
    t0 = spl.t[0]  # first knot is t0
    tf = spl.t[-1]  # last knot is tf
    # number of points in time vector so spacing is 0.01
    N = ceil((tf - t0)/0.01)
    t = np.linspace(t0, tf, N)  # time vector
    position = spl(t)
    # 3D trajectory plot
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    # plot control points (convert YX(-Z) -> NED)
    ax.plot(spl.c[:, 1], spl.c[:, 0], -spl.c[:, 2],
            '-o', label='control points')
    # plot spline (convert YX(-Z) -> NED)
    ax.plot(position[:, 1], position[:, 0], -position[:, 2],
            'b', label='spline')
    ax.legend()
    ax.set_xlabel('x', fontsize=16, rotation=0)
    ax.set_ylabel('y', fontsize=16, rotation=0)
    ax.set_zlabel('z', fontsize=16, rotation=0)
    #ax.set_xlim3d([-10, 10])
    plt.show()


if __name__ == "__main__":
    # initial and final time
    t0 = 0
    tf = 5
    order = 3
    knots = np.array([t0, t0, t0, t0,
                      (tf-t0)/3, 2*(tf-t0)/3,
                      tf, tf, tf, tf])
    # num control = num knots - order - 1
    ctrl_pts = np.array([[0, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1],
                         [0, 1, 1],
                         [1, 1, 0],
                         [1, 1, 1]])
    spl = BSpline(t=knots, c=ctrl_pts, k=order)
    plotSpline(spl)

