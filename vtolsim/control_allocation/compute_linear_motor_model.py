"""
    Simulate using equilibrium throttle commands to find a
    linear motor model.
"""
#! /usr/bin/env python3
import sys
sys.path.append('..')
sys.path.append('../viewers')

from tools.rotations import Quaternion2Euler, Quaternion2Rotation
from tools.msg_convert import *
from message_types.msg_controls import msgControls
from dynamics.vtol_dynamics import vtolDynamics
from dynamics.trim import *
import parameters.convergence_parameters as VTOL
import numpy as np
np.set_printoptions(precision=4, linewidth=200, suppress=True)

def compute_rotor_allocation_submatrix(servo0, Va_star=0.0, gamma_star=0.0):
    K = compute_K_least_squares(np.array([servo0]), Va_star=Va_star, gamma_star=gamma_star)

    KT1 = K[0]
    KT2 = K[1]
    KT3 = K[2]
    KM1 = K[3]
    KM2 = K[4]
    KM3 = K[5]

    q1 = VTOL.right_rotor_pos.reshape(-1)
    q2 = VTOL.left_rotor_pos.reshape(-1)
    q3 = VTOL.rear_rotor_pos.reshape(-1)

    allocation_matrix = np.array([
        [KT1, 0., KT2, 0., 0],
        [0, -KT1, 0, -KT2, -KT3],
        [-KM1, -KT1*q1[1], -KM2, -KT1*q2[1], -KT3*q3[1]],
        [KT1*q1[2], KT1*q1[0], KT2*q2[2], KT2*q2[0], KT3*q3[0]],
        [-KT1*q1[1], KM1, -KT2*q2[1], KM2, KM3]
    ])

    return allocation_matrix

def compute_K_least_squares(servo0_arr, Va_star=0.0, gamma_star=0.0):
    full_lhs = np.zeros((8*servo0_arr.size, 6))
    full_rhs = np.zeros((8*servo0_arr.size))
    for i in range(servo0_arr.size):
        full_lhs[8*i:8*(i+1),:], full_rhs[8*i:8*(i+1)] = compute_matrices(servo0_arr[i], force_KT1eqKT2=True, Va_star=Va_star, gamma_star=gamma_star)
    
    K, _, _, _ = np.linalg.lstsq(full_lhs, full_rhs, rcond=None)

    return K



def compute_matrices(servo0, force_KT1eqKT2=False, Va_star=0.0, gamma_star=0.0):
    # initialize elements of the architecture
    vtol = vtolDynamics()

    # use trim calculation to find hover for different desired rotor angles
    F = np.array([0., 0., -VTOL.mass * VTOL.gravity]) # inertial frame

    state_trim, delta_trim = compute_trim(vtol, Va_star, gamma_star, servo0 = servo0)
    R_b2i = Quaternion2Rotation(state_trim[6:10]) # body to inertial
    delta, theta_r = split_delta_trim(delta_trim)

    vtol._state = state_trim
    vtol._update_velocity_data()
    delta_trim_msg = np2msg_controls(delta_trim)
    forces_moments = vtol._forces_moments(delta_trim_msg)
    print(f"forces_moments = {forces_moments}")

    thrust_matrix = assemble_thrust_matrix(delta, theta_r)
    thrust_rhs = R_b2i.T @ F
    
    moment_matrix = assemble_moment_matrix(delta, theta_r)
    moment_rhs = np.zeros(3)

    # import pdb; pdb.set_trace()
    lhs = np.concatenate([thrust_matrix, moment_matrix], axis=0)
    rhs = np.concatenate([thrust_rhs, moment_rhs], axis=0)

    if force_KT1eqKT2:
        lhs = np.concatenate([lhs, np.array([[1., -1., 0., 0., 0., 0.], [0., 0., 0., 1., 1., 0.]])])
        rhs = np.concatenate([rhs, np.array([0., 0.])])
        # lhs = np.concatenate([lhs, np.array([[1., -1., 0., 0., 0., 0.]])])
        # rhs = np.concatenate([rhs, np.array([0.])])

    return lhs, rhs

def split_delta_trim(delta_trim):
    """
    convenience function to convert delta_trim into format that matches paper
    """
    delta_star = np.array([
        delta_trim.item(3), # right front rotor throttle
        delta_trim.item(4), # left front rotor throttle
        delta_trim.item(2), # rear rotor throttle
        delta_trim.item(0), # right elevon
        delta_trim.item(1)]) # left elevon

    theta_r_star = np.array([
        delta_trim.item(5), # right front rotor angle
        delta_trim.item(6)]) # left front rotor angle

    return delta_star, theta_r_star

def thrust_angle_vector(theta_r):
    return np.array([np.cos(theta_r), 0., -np.sin(theta_r)])

def assemble_thrust_matrix(delta, theta_r_vec):
    s_r1 = thrust_angle_vector(theta_r_vec[0])
    s_r2 = thrust_angle_vector(theta_r_vec[1])
    s_r3 = thrust_angle_vector(np.pi/2)

    thrust_matrix = np.zeros((3, 6))
    thrust_matrix[:,0] = delta[0] * s_r1
    thrust_matrix[:,1] = delta[1] * s_r2
    thrust_matrix[:,2] = delta[2] * s_r3
    # last 3 columns remain zeros

    return thrust_matrix

def assemble_moment_matrix(delta, theta_r_vec):
    s_r1 = thrust_angle_vector(theta_r_vec[0])
    s_r2 = thrust_angle_vector(theta_r_vec[1])
    s_r3 = thrust_angle_vector(np.pi/2)

    q1 = VTOL.right_rotor_pos.reshape(-1)
    q2 = VTOL.left_rotor_pos.reshape(-1)
    q3 = VTOL.rear_rotor_pos.reshape(-1)

    moment_matrix = np.zeros((3,6))
    moment_matrix[:,0] = delta[0] * np.cross(q1, s_r1)
    moment_matrix[:,1] = delta[1] * np.cross(q2, s_r2)
    moment_matrix[:,2] = delta[2] * np.cross(q3, s_r3)

    moment_matrix[:,3] = -delta[0] * s_r1
    moment_matrix[:,4] = -delta[1] * s_r2
    moment_matrix[:,5] = -delta[2] * s_r3
    
    return moment_matrix

###### Main 
def main():
    servo0_arr = np.array([np.pi/2, np.pi/4])
    print("Computing motor K vector using servo0 = ", servo0_arr)
    K = compute_K_least_squares(servo0_arr)

    A, b = compute_matrices(np.pi/2, force_KT1eqKT2=True)
    K1, _, _, _ = np.linalg.lstsq(A,b, rcond=None)
    A, b = compute_matrices(np.pi/4, force_KT1eqKT2=True)
    K2, _, _, _ = np.linalg.lstsq(A,b, rcond=None)
    A, b = compute_matrices(0., force_KT1eqKT2=True)
    K3, _, _, _ = np.linalg.lstsq(A,b, rcond=None)

    print("K = ", K)
    print("K1 = ", K1)
    print("K2 = ", K2)
    print("K4 = ", K3)

if __name__ == "__main__":
    main()