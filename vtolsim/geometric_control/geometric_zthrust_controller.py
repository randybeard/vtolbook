"""
Geometric Controller for vtolsim
"""

import sys
import numpy as np
from scipy.linalg import expm
sys.path.append('..')
from tools.rotations import Quaternion2Rotation, vee, hat
import parameters.convergence_parameters as VTOL
import parameters.geometric_control_parameters as GEOM


class GeometricZThrustController:
    """
    Geometric tracking, but fixes thrust to be in -z direction and removes pitch optimization
    """
    def __init__(self, time_step, seed_optimizer=True, constrain_pitch_rate=True):
        self.seed_optimizer = seed_optimizer
        self.constrain_pitch_rate = constrain_pitch_rate

        self.alpha_p_prev = 0.0
        self.T_prev = None

        self.time_step = time_step

        self.perr_integral = np.zeros(3)
        self.perr_d0_delay1 = np.zeros(3)

        self.name = "ZThrust"

    def update(self, state, trajectory_flag):
        F_d, R_d2i, omega_d = self.trajectory_follower(state, trajectory_flag)

        # attitude control
        omega_c = attitude_controller(state, R_d2i, omega_d)

        # desired position/velocity for plotting
        pd = trajectory_flag[0:3,0]
        vd_i = trajectory_flag[0:3,1]
        vd_b = R_d2i.T @ vd_i

        return F_d, R_d2i, omega_c, pd, vd_b

    def trajectory_follower(self, state, trajectory_flag):
        # pi_d0 = true position in inertial frame, 0th derivative
        pi_d0 = state[0:3].reshape(-1)
        q_b2i = state[6:10].reshape(-1)
        R_b2i = Quaternion2Rotation(q_b2i)
        pi_d1 = R_b2i @ state[3:6].reshape(-1)

        # pd_d0 = position desired, 0th derivative
        pd_d0 = trajectory_flag[0:3,0]
        pd_d1 = trajectory_flag[0:3,1]
        pd_d2 = trajectory_flag[0:3,2]
        pd_d3 = trajectory_flag[0:3,3]

        psi_d0 = trajectory_flag[3,0]
        psi_d1 = trajectory_flag[3,1]

        # errors
        perr_d0 = pi_d0 - pd_d0
        # perr_d0 = np.clip(perr_d0, -GEOM.perr_d0_sat, GEOM.perr_d0_sat)
        perr_d1 = pi_d1 - pd_d1

        # position error integral
        self.perr_integral = self.perr_integral + .5 * self.time_step * (perr_d0 + self.perr_d0_delay1)
        self.perr_d0_delay1 = perr_d0

        e3 = np.array([0., 0., 1.])

        # Inertial frame applied force vector
        fI = (VTOL.mass
                * (pd_d2
                - VTOL.gravity*e3
                - GEOM.Kp @ perr_d0
                - GEOM.Kd @ perr_d1))

        # normalized
        fI_mag = np.linalg.norm(fI)

        s_phi = np.array([
            np.cos(psi_d0),
            np.sin(psi_d0),
            0.0])

        # Body frame z axis in inertial frame
        Rz_b = -fI/fI_mag

        # Body frame y axis in inertial frame
        Ry_b_dir = np.cross(Rz_b, s_phi)
        Ry_b_mag = np.linalg.norm(Ry_b_dir)
        Ry_b = Ry_b_dir/Ry_b_mag

        # Body frame z axis in inertial frame
        Rx_b = np.cross(Ry_b, Rz_b)

        # Rotation matrix from desired to inertial
        R_d = np.column_stack([Rx_b, Ry_b, Rz_b])

        F_x = 0.0
        F_z = R_b2i[:,2] @ fI

        F_d = np.array([F_x, F_z])

        # derivative of force vector
        fI_d1 = VTOL.mass * (pd_d3
                - GEOM.Kp * perr_d1
                - GEOM.Kd * (-GEOM.Kp*perr_d0 - GEOM.Kd*perr_d1))

        # derivative of yaw vector
        s_phi_d1 = np.array([
            -psi_d1 * np.sin(psi_d0),
            psi_d1 * np.cos(psi_d0),
            0.0])

        # derivative of z-axis
        Rz_b_d1 = fI_d1/fI_mag - fI * (fI_d1 @ fI)/(fI_mag**3)

        # derivative of y-axis
        Ry_b_d1 = np.cross(Rz_b_d1, s_phi) + np.cross(Rz_b, s_phi_d1)

        # derivative of x-axis
        Rx_b_d1 = np.cross(Ry_b_d1, Rz_b) + np.cross(Ry_b, Rz_b_d1)

        # derivative of Rd
        R_d_d1 = np.column_stack([Rx_b_d1, Ry_b_d1, Rz_b_d1])

        # Angular rates
        omega_d = vee(R_d.T @ R_d_d1)

        return F_d, R_d, omega_d
    
def attitude_controller(state, R_d2i, omega_d):
    q_b2i = state[6:10]
    R_b2i = Quaternion2Rotation(q_b2i)
    R_d2b = R_b2i.T @ R_d2i
    if .5*(np.trace(np.eye(3) - R_d2b)) >= 2:
        print("Attitude controller assumptions violated")
        print("R_p2b = ", R_p2b)

    omega_c = R_d2b @ omega_d + GEOM.omega_Kp @ vee(antisymmetric(R_d2b))

    return omega_c

def antisymmetric(mat):
    """
    returns the antisymmetric part of a matrix
    """
    return (1./2.)*(mat - mat.T)