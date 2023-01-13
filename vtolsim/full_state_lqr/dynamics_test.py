import sys
sys.path.append('..')
import numpy as np


np.set_printoptions(precision=4, linewidth=200, suppress=True)

from dynamics.vtol_dynamics import vtolDynamics
vtol = vtolDynamics()
from tools.quaternions import *
from trajectory_planning.differential_flatness import DifferentialFlatness
from trajectory_planning.trajectory import XYZSinusoid
from tools.msg_convert import *


import lqr_dynamics

x = np.array([[0., 0., 0., 10., 0., 0., 1., 0., 0., 0., 0., 0., 0., np.pi/2., np.pi/2.]]).T
# x_des = np.array([[0, 100., 
# u = np.array([[0.0, 1.0, 0.0, 0.0, 0.0]]).T

# x =  np.array([[-30., 100.,   0.,   0.,   0.,   0.,   0.998,    0.,       0.,       0.0628]]).T
x_des =  np.array([[-12.5333,  99.2115,   0.,       6.0858,   1.5626,   0.,       0.998,    0.,       0.,       0.0628]]).T

df_traj = DifferentialFlatness(XYZSinusoid(100, 100, 0, 100, 100, 0, 0, np.pi/2, 0))

u =  np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]).T
delta  = lqr_dynamics.mixer(u[0:2])
a = np.empty(0)
x_dot = np.empty((3,0))
for inc in np.arange(0.0, 1., .01):
    # x[3][0] = inc
    u[1][0] = inc
    # f = lqr_dynamics.forces(x, u)
    vtol._state = x
    vtol._update_true_state()
    vtol._update_velocity_data()
    true_fm = vtol._forces_moments(u, delta) 
    x_dot = np.append(x_dot, true_fm[0:3], axis =1)
    a = np.append(a, inc)

import matplotlib.pyplot as plt
plt.figure()
plt.plot(a, x_dot[0][:], label = 'fx')
plt.plot(a, x_dot[1][:], label = 'fy')
plt.plot(a, x_dot[2][:], label = 'fz')
plt.legend()
plt.title("vtol_dynamics")

a = np.empty(0)
x_dot = np.empty((3,0))
for inc in np.arange(0.0, 1., .01):
    # x[3][0] = inc
    u[1][0] = inc
    f = lqr_dynamics.forces(x, u)
    x_dot = np.append(x_dot, f, axis =1)
    a = np.append(a, inc)

import matplotlib.pyplot as plt
plt.figure()
plt.plot(a, x_dot[0][:], label = 'fx')
plt.plot(a, x_dot[1][:], label = 'fy')
plt.plot(a, x_dot[2][:], label = 'fz')
plt.legend()
plt.title("lqr_dynamics")
plt.show()

# u =  np.array([[0., 0., 0., 0., 0.]]).T
# x_tilde = state_boxMinus(x_des, x)
# A = lqr_dynamics.df_dx(x_tilde, x_des, u, df_traj)
# print("A = \n", A)
# B = lqr_dynamics.df_du(x_tilde, x_des, u, df_traj)
# print("B = \n", B)

# vtol._state = x
# vtol._update_true_state()
# vtol._update_velocity_data()
# true_fm = vtol._forces_moments(u, delta) 
# lqr_fm = lqr_dynamics.forces(x, u)

# print("true_fm = ", true_fm[0:3].T)
# print("lqr_fm  = ", lqr_fm.T)

# true_deriv = vtol._derivatives(x, true_fm, delta)
# lqr_deriv = lqr_dynamics.f(x, u)

# print("true_deriv = ", true_deriv.T)
# print("lqr_deriv  = ", lqr_deriv.T)


