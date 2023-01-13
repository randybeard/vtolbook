"""
Geometric Controller for vtolsim
"""

import sys
import numpy as np
from scipy.linalg import expm
sys.path.append('..')
from tools.rotations import Quaternion2Euler, Quaternion2Rotation, Rotation2Euler, vee, hat
import parameters.convergence_parameters as VTOL
import parameters.geometric_control_parameters as GEOM
from geometric_control.optimal_pitch import compute_thrust, find_pitch_thrust, find_thrust_from_theta


class GeometricController:
    def __init__(self, time_step, seed_optimizer=True, constrain_pitch_rate=True, theta_0=0.):
        self.seed_optimizer = seed_optimizer
        self.constrain_pitch_rate = constrain_pitch_rate

        self.T_opt_prev = None
        self.theta_opt_prev = None
        self.theta_cmd_prev = theta_0

        self.time_step = time_step

        self.perr_integral = np.zeros(3)
        self.perr_d0_delay1 = np.zeros(3)

        self.name = "PitchOptimized" + GEOM.optimal_pitch_method.name + "_" + GEOM.aero_type.name

    def update(self, state, trajectory_flag, transition=False, time_step=None):
        if time_step is None:
            time_step = self.time_step

        F_d, R_d2i, omega_d_nopitch = self.trajectory_follower(state, trajectory_flag, time_step)

        # find Va and gamma
        vd_i = trajectory_flag[0:3,1]
        vd_d = R_d2i.T @ vd_i
        gamma = np.arctan2(-vd_d[2], vd_d[0])
        Va = np.linalg.norm(vd_d)

        # find optimal thrust and pitch
        theta_opt_p2d, T_opt_d_p = find_pitch_thrust(vd_d, F_d, previous_theta=self.theta_opt_prev, model=GEOM.aero_type, method=GEOM.optimal_pitch_method, transition=transition)

        # filter theta
        if transition:
            T_d_p = T_opt_d_p
            theta_p2d = theta_opt_p2d
        elif self.constrain_pitch_rate and np.abs(self.theta_cmd_prev - theta_opt_p2d) > GEOM.delta_theta_max:
                theta_p2d = self.theta_cmd_prev + np.sign(theta_opt_p2d - self.theta_cmd_prev)*GEOM.delta_theta_max
                # make sure thrust matches pitch angle
                T_d_p = find_thrust_from_theta(vd_d, F_d, theta_p2d, model=GEOM.aero_type)
        else:
            T_d_p = T_opt_d_p
            theta_p2d = theta_opt_p2d

        self.T_opt_prev = T_opt_d_p
        self.theta_opt_prev = theta_opt_p2d

        self.theta_cmd_prev = theta_p2d

        # y_d2i = R_d2i[:,1]
        # R_p2d = expm(-hat(theta_p2d*y_d2i)).T
        R_p2d = expm(-hat(theta_p2d*np.array([0., 1., 0.]))).T

        R_p2i = R_d2i @ R_p2d

        omega_p2i_p = R_p2d.T @ omega_d_nopitch
        # omega_p2i_p = omega_d_nopitch

        B = np.array([[1., 0.], [0., 0.], [0., 1.]])
        q_b2i = state[6:10].reshape(-1)
        R_b2i = Quaternion2Rotation(q_b2i)
        T_d_in_b = B.T @ R_b2i.T @ R_d2i @ R_p2d @ B @ T_d_p
        # T_d_in_b = T_d_in_p

        # attitude control
        omega_c = attitude_controller(state, R_p2i, omega_p2i_p)


        # desired position/velocity for plotting
        pd = trajectory_flag[0:3,0]
        vd_b_pitch = R_p2i.T @ vd_i

        return T_d_in_b, R_p2i, omega_c, pd, vd_b_pitch, omega_p2i_p

    def trajectory_follower(self, state, trajectory_flag, time_step):
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
        perr_d1 = pi_d1 - pd_d1

        # position error integral
        self.perr_integral = self.perr_integral + .5 * time_step * (perr_d0 + self.perr_d0_delay1)
        self.perr_d0_delay1 = perr_d0

        e3 = np.array([0., 0., 1.])

        # Inertial frame applied force vector
        fI = (VTOL.mass
                * (pd_d2
                - VTOL.gravity*e3
                - GEOM.Kp @ perr_d0
                - GEOM.Kd @ perr_d1
                - GEOM.Ki @ self.perr_integral))

        # normalized
        fI_mag = np.linalg.norm(fI)

        # Body frame x axis in inertial frame
        Rx_b = np.array([
            np.cos(psi_d0),
            np.sin(psi_d0),
            0.0])

        # Body frame y axis in inertial frame
        Ry_b_used_fI = True
        Ry_b_dir = np.cross(Rx_b, fI)
        Ry_b_mag = np.linalg.norm(Ry_b_dir)
        if Ry_b_mag < .1:
            Ry_b_dir = np.cross(Rx_b, e3)
            Ry_b_mag = np.linalg.norm(Ry_b_dir)
            Ry_b_used_fI = False
        Ry_b = Ry_b_dir/Ry_b_mag

        # Body frame z axis in inertial frame
        Rz_b = np.cross(Rx_b, Ry_b)

        # Rotation matrix from desired to inertial
        R_d = np.column_stack([Rx_b, Ry_b, Rz_b])

        F_x = Rx_b @ fI
        F_z = Rz_b @ fI

        F_d = np.array([F_x, F_z])

        # derivative of force vector
        # TODO: add integral term here
        fI_d1 = VTOL.mass * (pd_d3
                - GEOM.Kp * perr_d1
                - GEOM.Kd * (-GEOM.Kp*perr_d0 - GEOM.Kd*perr_d1))

        # derivative of x-axis
        Rx_b_d1 = np.array([
            -psi_d1 * np.sin(psi_d0),
            psi_d1 * np.cos(psi_d0),
            0.0])

        # derivative of y-axis
        if Ry_b_used_fI:
            Ry_b_dir_d1 = np.cross(Rx_b_d1, fI) + np.cross(Rx_b, fI_d1)
        else:
            Ry_b_dir_d1 = np.cross(Rx_b_d1, e3)

        Ry_b_d1 = ((Ry_b_dir_d1/Ry_b_mag) - Ry_b*(Ry_b_dir_d1.T @ Ry_b)/(Ry_b_mag**3))

        # derivative of z-axis
        Rz_b_d1 = np.cross(Rx_b_d1, Ry_b) + np.cross(Rx_b, Ry_b_d1)

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
        print("R_d2b = ", R_d2b)

    omega_c = R_d2b @ omega_d + GEOM.omega_Kp @ vee(antisymmetric(R_d2b))

    return omega_c

def antisymmetric(mat):
    """
    returns the antisymmetric part of a matrix
    """
    return (1./2.)*(mat - mat.T)
