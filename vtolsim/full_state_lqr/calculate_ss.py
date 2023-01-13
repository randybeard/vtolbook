#! /usr/bin/env python3
"""
Compute the state space models for the forces/moments controller
and the system dynamics
"""

import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from tools.rotations import Euler2Quaternion, Quaternion2Euler
from tools.transfer_function import transferFunction
from dynamics.vtol_dynamics import vtolDynamics
import parameters.convergence_parameters as VTOL
from parameters.simulation_parameters import ts_simulation as Ts
from hover_controller.compute_delta import compute_delta, compute_forces
import parameters.simulation_parameters as SIM
from dynamics.wind_simulation import windSimulation
from message_types.msg_controls import msgControls
from dynamics.trim import *
from tools.msg_convert import np2msg_controls

class calculate_ss:
    def __init__(self):
        self.vtol = vtolDynamics()

        
    def compute_ss_model(self, x, u):
        if (np.size(x,0) == 14):
            x = quaternion_state(x)
        x_euler = euler_state(x)
        A = self.df_dx(x_euler, u)
        B = self.df_du(x_euler, u)
        return A, B, x_euler


    def f_euler(self, x_euler, delta_input):
        # return 12x1 dynamics (as if state were Euler state)
        # compute f at euler_state
        x_quat = quaternion_state(x_euler)
        self.vtol._state = x_quat
        self.vtol._update_velocity_data()

        delta_msg = np2msg_controls(delta_input) 
        forces_moments = self.vtol._forces_moments(delta_msg)
        f = self.vtol._derivatives(x_quat, forces_moments, delta_msg)
        # f_euler will be f, except for the attitude states
        f_euler_ = euler_state(f)

        # need to correct attitude states by multiplying f by
        # partial of Quaternion2Euler(quat) with respect to quat
        # compute partial Quaternion2Euler(quat) with respect to quat
        eps = 0.001  # epsilon used to approximate derivatives
        e = x_quat[6:10]
        phi = x_euler.item(6)
        theta = x_euler.item(7)
        psi = x_euler.item(8)
        dTheta_dquat = np.zeros((3,4))
        for j in range(0, 4):
            tmp = np.zeros((4, 1))
            tmp[j][0] = eps
            #e_eps = (e + tmp)
            e_eps = (e + tmp) / np.linalg.norm(e + tmp)
            phi_eps, theta_eps, psi_eps = Quaternion2Euler(e_eps)
            dTheta_dquat[0][j] = (phi_eps - phi) / eps
            dTheta_dquat[1][j] = (theta_eps - theta) / eps
            dTheta_dquat[2][j] = (psi_eps - psi) / eps
        # Multiply f by
        # partial of Quaternion2Euler(quat) with respect to quat
        f_euler_[6:9] = np.copy(dTheta_dquat @ f[6:10])
        return f_euler_

    def df_dx(self, x_euler, u_input):
        # convert u_input into delta commands
        delta_input = compute_delta(u_input)
        # take partial of f_euler with respect to x_euler
        eps = 0.01  # deviation
        A = np.zeros((14, 14))  # Jacobian of f wrt x
        f = self.f_euler(x_euler, delta_input)
        for i in range(0, 14):
            x_eps = np.copy(x_euler)
            x_eps[i][0] += eps
            f_eps = self.f_euler(x_eps, delta_input)
            df = (f_eps - f) / eps
            A[:,i] = df[:,0]
        return A

    def df_du(self, x_euler, u_input):
        # convert u_input into delta command
        delta_input = compute_delta(u_input)
        # get df_ddelta
        B1 = self.df_ddelta(x_euler, delta_input)
        # get df_du
        B2 = self.ddelta_du(u_input)

        return B1 @ B2


    def df_ddelta(self, x_euler, delta_input):
        # take partial of f_euler with respect to delta
        eps = 0.01  # deviation
        B = np.zeros((14, 7))  # Jacobian of f wrt delta
        f = self.f_euler(x_euler, delta_input)
        for i in range(0, 7):
            delta_input_eps = np.copy(delta_input)
            delta_input_eps[i, 0] += eps
            f_eps = self.f_euler(x_euler, delta_input_eps)
            df = (f_eps - f) / eps
            B[:,i] = df[:,0]
        return B

    def ddelta_du(self, u_input):
        # take partial of compute_delta with respect to u_input
        eps = 0.01  # deviation
        B = np.zeros((7,8))  
        g = compute_delta(u_input)
        for i in range(0, 8):
            u_input_eps = np.copy(u_input)
            u_input_eps[i, 0] += eps
            g_eps = compute_delta(u_input_eps)
            dg = (g_eps - g) / eps
            B[:,i] = dg[:,0]
        return B

def euler_state(x_quat):
    # convert state x with attitude represented by quaternion
    # to x_euler with attitude represented by Euler angles
    x_euler = np.zeros((14,1))
    x_euler[0:6] = np.copy(x_quat[0:6])  # copy position, velocity
    phi, theta, psi = Quaternion2Euler(x_quat[6:10])
    x_euler[6] = phi
    x_euler[7] = theta
    x_euler[8] = psi
    x_euler[9:14] = np.copy(x_quat[10:15]) # copy angular rate
    return x_euler

def quaternion_state(x_euler):
    # convert state x_euler with attitude represented by Euler angles
    # to x_quat with attitude represented by quaternions
    x_quat = np.zeros((15,1))
    x_quat[0:6] = np.copy(x_euler[0:6])  # copy position, velocity
    phi = x_euler.item(6)
    theta = x_euler.item(7)
    psi = x_euler.item(8)
    quat = Euler2Quaternion(phi, theta, psi)
    x_quat[6:10] = quat
    x_quat[10:15] = np.copy(x_euler[9:14]) # copy angular rate
    return x_quat


def ssAtTrimpoint(Va_desired, gamma_desired_rad):
    # initialize elements of the architecture
    wind = windSimulation()
    vtol = vtolDynamics()

    # initialize command message
    delta = msgControls()

    #calculate_trim
    state_trim, delta_trim = compute_trim(vtol, Va_desired, gamma_desired_rad)
    force_trim = compute_forces(delta_trim)
    ss_calc_ = calculate_ss()
    A, B = ss_calc_.compute_ss_model(state_trim, force_trim)
    return A, B

if __name__ == "__main__":

    A0, B0 = ssAtTrimpoint(0.0, np.radians(0.0))
    np.set_printoptions(precision=4, linewidth=200, suppress=True)
    # print("A0:\n", A0)
    # print("B0:\n", B0)
    for i in np.arange(1.0, 20.0):
        A, B = ssAtTrimpoint(i, np.radians(0.0))
        print("Va = {}, sum(A0-A) = {}, sum(B0-B) = {}".format(i, np.sum(A0-A), np.sum(B0-B)))
    for i in np.arange(1.0, 20.0):
        A, B = ssAtTrimpoint(0.0, np.radians(i))
        print("gamma = {}, sum(A0-A) = {}, sum(B0-B) = {}".format(i, np.sum(A0-A), np.sum(B0-B)))


    # print("A:\n", A)
    # print("B:\n", B)

    # import control
    # ctrb = control.ctrb(A, B)
    # print("Ctrb:\n", ctrb)
    # print("Rank ctrb: ", np.linalg.matrix_rank(ctrb), "\tRank A: ", np.linalg.matrix_rank(A))
