#! /usr/bin/env python3

import sys
sys.path.append("..")
import numpy as np
import matplotlib.pyplot as plt

from control_allocation_cls import ControlAllocation
from tools.msg_convert import msg_controls2np
from dynamics.vtol_dynamics import vtolDynamics

ca_hover = ControlAllocation(servo0=np.pi/2, Va=0., gamma=0.)

vtol = vtolDynamics()

def runCase(thrust, torque, Va, alpha):

    ca_hover_delta = ca_hover.update(thrust, torque, Va)
    # print("ca_hover_delta = ", msg_controls2np(ca_hover_delta).reshape(-1))

    vtol._Va = Va
    vtol._alpha = np.deg2rad(alpha)
    vtol.v_air = np.array([Va*np.cos(vtol._alpha), 0., Va*np.sin(vtol._alpha)])
    vtol._state[13] = ca_hover_delta.servo_right
    vtol._state[14] = ca_hover_delta.servo_left

    hover_forces_moments = vtol._forces_moments(ca_hover_delta)
    # print("hover moments actual = ", hover_forces_moments[3:,0])
    # print("hover moments expect = ", torque)
    # print("hover thrust actual = ", [vtol.total_thrust[0], vtol.total_thrust[2]])
    # print("hover thrust expect = ", thrust)
    return hover_forces_moments


# runCase(thrust = np.array([0., -9.81]), torque = np.array([0., 0., 0.]), Va = 0.0, alpha = 0.0)
# runCase(thrust = np.array([.0, -9.81]), torque = np.array([.1, 0., 0.]), Va = 0.0, alpha = 0.0)
# runCase(thrust = np.array([.0, -9.81]), torque = np.array([0., .1, 0.]), Va = 0.0, alpha = 0.0)
# runCase(thrust = np.array([.0, -9.81]), torque = np.array([0., 0., .1]), Va = 0.0, alpha = 0.0)
# runCase(thrust = np.array([.0, -9.81]), torque = np.array([-.1, 0., 0.]), Va = 0.0, alpha = 0.0)
# runCase(thrust = np.array([.0, -9.81]), torque = np.array([0., -.1, 0.]), Va = 0.0, alpha = 0.0)
# runCase(thrust = np.array([.0, -9.81]), torque = np.array([0., 0., -.1]), Va = 0.0, alpha = 0.0)

s = np.arange(0., 2*np.pi, .01)
tau_vec = .5*np.sin(s)
true_tau_x = []
for tau in tau_vec:
    # fm = runCase(np.array([0., -9.81]), np.array([tau, 0., 0.]), 0., 0.)
    # fm = runCase(np.array([0., -9.81]), np.array([tau, tau, tau]), 0., 0.)
    fm = runCase(np.array([3.5, -.5]), np.array([tau, 0., 0.]), 20., 5.)
    # fm = runCase(np.array([3.5, -.5]), np.array([tau, tau, tau]), 20., 5.)
    true_tau_x.append(fm.item(3))

true_tau_y = []
for tau in tau_vec:
    # fm = runCase(np.array([0., -9.81]), np.array([0., tau, 0.]), 20., 5.)
    # fm = runCase(np.array([0., -9.81]), np.array([tau, tau, tau]), 0., 0.)
    fm = runCase(np.array([3.5, -.5]), np.array([0., tau, 0.]), 20., 5.)
    # fm = runCase(np.array([3.5, -.5]), np.array([tau, tau, tau]), 20., 5.)
    true_tau_y.append(fm.item(4))

true_tau_z = []
for tau in tau_vec:
    # fm = runCase(np.array([0., -9.81]), np.array([0., 0., tau]), 0., 0.)
    # fm = runCase(np.array([0., -9.81]), np.array([tau, tau, tau]), 0., 0.)
    fm = runCase(np.array([3.5, -.5]), np.array([0., 0., tau]), 20., 5.)
    # fm = runCase(np.array([3.5, -.5]), np.array([tau, tau, tau]), 20., 5.)
    true_tau_z.append(fm.item(5))

fig, ax = plt.subplots(3,1)
ax[0].plot(s,tau_vec, label='desired')
ax[0].plot(s,true_tau_x, label='actual')
ax[0].legend()
ax[1].plot(s,tau_vec, label='desired')
ax[1].plot(s,true_tau_y, label='actual')
ax[1].legend()
ax[2].plot(s,tau_vec, label='desired')
ax[2].plot(s,true_tau_z, label='actual')
ax[2].legend()
plt.show()

runCase(np.array([3.3423, -.9511]), np.array([0.0013, .5235, -0.0098]), 16.075, 5.69)