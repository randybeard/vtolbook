"""
draw a random B-spline surface
        5/2/2022 - RWB
"""
import numpy as np
import splipy as sp
import matplotlib.pyplot as plt
from bspline_tools import uniform_clamped_knots
# from scipy.interpolate import BSpline
# import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def draw_basis(order=3, M=4):
    # order of B-spline (3==cubic -> different than splipy def)
    # domain is [0, M]
    knots = uniform_clamped_knots(k=order, M=M)
    basis = sp.BSplineBasis(order=order+1, knots=knots)
    # 150 uniformly spaced evaluation points on the domain (0,M)
    t = np.linspace(0, M, 150)
    # evaluate *all* basis functions on *all* points t. The returned variable B is a matrix
    B = basis.evaluate(t)  # B.shape = (150,6), 150 visualization points, 6 basis functions
    dB = basis.evaluate(t, d=1)  # compute the first derivatives
    ddB = basis.evaluate(t, d=2)  # compute the second derivatives

    # plot the basis functions
    plt.plot(t, B)
    #plt.plot(t, dB)
    # plt.plot(t, ddB)
    plt.show()

def draw_random_surface(order=2, M=10, N=10,
                        Xmin=-3.14159, Xmax=3.14159,
                        Ymin=-2*3.14159, Ymax=2*3.14159):
    # define the spline surface
    knots_x = uniform_clamped_knots(k=order, M=M)
    knots_y = uniform_clamped_knots(k=order, M=N)
    basis_x = sp.BSplineBasis(order + 1, knots_x)
    basis_y = sp.BSplineBasis(order + 1, knots_y)
    C = np.random.rand(M + order, N + order)
    surface = sp.Surface(basis_x, basis_y,
                         np.reshape(C, ((M + order) * (N + order), 1)))
    # plot the spline surface
    x = np.linspace(Xmin, Xmax, 10 * M)
    y = np.linspace(Ymin, Ymax, 10 * N)
    s = surface(M*(x-Xmin)/(Xmax-Xmin),
                N*(y-Ymin)/(Ymax-Ymin))  # surface points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.xlabel('x')
    plt.ylabel('y')
    x_pts, y_pts = np.meshgrid(x, y)  # grid points
    ax.plot_surface(x_pts, y_pts, s[:, :, 0])
    plt.savefig('random_spline_surface.pdf')
    plt.show()


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def draw_surface_basis(order=2):
    M = order + 5
    N = order + 5
    # define the spline surface
    knots_x = uniform_clamped_knots(k=order, M=M)
    knots_y = uniform_clamped_knots(k=order, M=N)
    basis_x = sp.BSplineBasis(order + 1, knots_x)
    basis_y = sp.BSplineBasis(order + 1, knots_y)
    C = np.zeros((M + order, N + order))
    C[order+1, order+1] = 1
    surface = sp.Surface(basis_x, basis_y,
                         np.reshape(C, ((M + order) * (N + order), 1)))
    # plot the spline surface
    x = np.linspace(0, M, 10 * M)
    y = np.linspace(0, N, 10 * N)
    s = surface(x, y)  # surface points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #ax.set_aspect('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    x_pts, y_pts = np.meshgrid(x, y)  # grid points
    ax.plot_surface(x_pts, y_pts, s[:, :, 0], cmap=plt.cm.cividis)
    ax.axes.set_xlim3d(left=0, right=7)
    ax.axes.set_ylim3d(bottom=0, top=7)
    ax.axes.set_zlim3d(bottom=0, top=1)
    #set_axes_equal(ax)

    #plt.savefig('spline_basis_function.pdf')
    plt.show()


if __name__ == "__main__":
    #draw_basis(order=3, M=4)
    #draw_random_surface(order=2, M=10, N=10)
    draw_surface_basis(order=0)
