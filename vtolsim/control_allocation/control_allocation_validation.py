#! /usr/bin/env python3

import sys
sys.path.append("..")
import numpy as np

from control_allocation_cls import ControlAllocation
from tools.msg_convert import msg_controls2np
from dynamics.vtol_dynamics import vtolDynamics

ca_hover = ControlAllocation(servo0=np.pi/2, Va=0., gamma=0.)
ca_full_wing = ControlAllocation(servo0=0., Va=20., gamma=0.)
ca_half_wing = ControlAllocation(servo0=np.pi/4, Va=10., gamma=0.)

vtol = vtolDynamics()

def runCase(num, name, thrust, torque, Va, alpha):
    print(f"\nCase {num}: {name}")

    ca_hover_delta = ca_hover.update(thrust, torque, Va)
    print("ca_hover_delta = ", msg_controls2np(ca_hover_delta).reshape(-1))
    ca_full_delta = ca_full_wing.update(thrust, torque, Va)
    print("ca_full_delta = ", msg_controls2np(ca_full_delta).reshape(-1))
    ca_half_delta = ca_half_wing.update(thrust, torque, Va)
    print("ca_half_delta = ", msg_controls2np(ca_half_delta).reshape(-1))

    vtol._Va = Va
    vtol._alpha = np.deg2rad(alpha)
    vtol.v_air = np.array([Va*np.cos(alpha), 0., Va*np.sin(alpha)])

    hover_forces_moments = vtol._forces_moments(ca_hover_delta)
    print("hover_forces_moments = ", hover_forces_moments.reshape(-1))
    print("hover thrust = ", vtol.total_thrust)
    full_forces_moments = vtol._forces_moments(ca_full_delta)
    print("full_forces_moments = ", full_forces_moments.reshape(-1))
    print("full thrust = ", vtol.total_thrust)
    half_forces_moments = vtol._forces_moments(ca_half_delta)
    print("half_forces_moments = ", half_forces_moments.reshape(-1))
    print("half thrust = ", vtol.total_thrust)


runCase(1, "Hover", thrust = np.array([0., -9.81]), torque = np.array([0., 0., 0.]), Va = 0.0, alpha = 10.0)
runCase(2, "Half Transition", thrust = np.array([3., -1.]), torque = np.array([0., 0., 0.]), Va = 10.0, alpha = 10.0)
runCase(3, "Forward", thrust = np.array([4., -.5]), torque = np.array([0., 0., 0.]), Va = 20.0, alpha = 10.0)
runCase(4, "Forward with x torque", thrust = np.array([4., -.5]), torque = np.array([1., 0., 0.]), Va = 20.0, alpha = 10.0)
runCase(5, "Forward with y torque", thrust = np.array([4., -.5]), torque = np.array([0., 1., 0.]), Va = 20.0, alpha = 10.0)
runCase(6, "Forward with z torque", thrust = np.array([4., -.5]), torque = np.array([0., 0., 1.]), Va = 20.0, alpha = 10.0)
runCase(7, "Hover with x torque", thrust = np.array([.0, -9.81]), torque = np.array([1., 0., 0.]), Va = 0.0, alpha = 0.0)
runCase(8, "Hover with y torque", thrust = np.array([.0, -9.81]), torque = np.array([0., 1., 0.]), Va = 0.0, alpha = 0.0)
runCase(9, "Hover with z torque", thrust = np.array([.0, -9.81]), torque = np.array([0., 0., 1.]), Va = 0.0, alpha = 0.0)