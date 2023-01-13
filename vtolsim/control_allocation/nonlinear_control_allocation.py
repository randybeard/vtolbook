import re
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import linprog, minimize, NonlinearConstraint

import parameters.convergence_parameters as VTOL
import parameters.control_allocation_parameters as CA
from tools.msg_convert import *
from message_types.msg_controls import msgControls
from control_allocation.control_allocation_cls import ControlAllocation
from tools.rotations import Quaternion2Euler

CA_ROTOR_RIGHT = 0
CA_ROTOR_LEFT = 1
CA_ROTOR_REAR = 2
CA_SERVO_RIGHT = 3
CA_SERVO_LEFT = 4
CA_ELEVON_RIGHT = 5
CA_ELEVON_LEFT = 6

class NonlinearControlAllocation(ControlAllocation):
    def __init__(self):
        self.previous_solution = CA.init_actuators
        self.max_iter = CA.max_iter
        self.actuator_bounds = [(0.0,  1.0),
                                (0.0,  1.0),
                                (0.0,  1.0),
                                (0.0,  VTOL.servo_max),
                                (0.0,  VTOL.servo_max),
                                (-1.0, 1.0),
                                (-1.0, 1.0)]

    def update(self, thrust, torques, state, airspeed):

        thrust_torque_desired = np.concatenate([thrust, torques], axis=0).reshape(-1)
        v_body = state[3:6]
        # factor_min = 0.0
        # if airspeed < 15.0:
        #     upward_thrust_limiting_factor = 1.0
        # elif airspeed < 25.0:
        #     upward_thrust_limiting_factor = (25.0 - airspeed) * (.1 - factor_min) + factor_min
        # else:
        #     upward_thrust_limiting_factor = factor_min

        # # self.actuator_bounds[CA_SERVO_LEFT] = (0.0, VTOL.servo_max * upward_thrust_limiting_factor)
        # # self.actuator_bounds[CA_SERVO_RIGHT] = (0.0, VTOL.servo_max * upward_thrust_limiting_factor)
        # self.actuator_bounds[CA_ROTOR_REAR] = (0.0, upward_thrust_limiting_factor)

        actuator_commands = self._compute_nonlinear_optimization(thrust_torque_desired, v_body, airspeed)

        return self._formulate_ctrl_msg(actuator_commands)

    def _compute_nonlinear_optimization(self, thrust_torque_desired, v_body, airspeed):

        x0 = self.previous_solution
        
        # Non linear optimizer gets optimization output and gradient from nonlinear_ctrl_optimization output
        res = minimize(
            nonlinear_ctrl_optimization, 
            x0,
            args=(thrust_torque_desired, v_body, airspeed, x0),
            bounds=self.actuator_bounds,
            jac=True,
            options={'maxiter': self.max_iter})
        self.previous_solution = res.x
        return res.x

    def _formulate_ctrl_msg(self, actuator_commands):
        ctrl_msg = msgControls()

        ctrl_msg.throttle_right = actuator_commands[CA_ROTOR_RIGHT]
        ctrl_msg.throttle_left = actuator_commands[CA_ROTOR_LEFT]
        ctrl_msg.throttle_rear = actuator_commands[CA_ROTOR_REAR]

        ctrl_msg.servo_right = actuator_commands[CA_SERVO_RIGHT]
        ctrl_msg.servo_left = actuator_commands[CA_SERVO_LEFT]

        ctrl_msg.elevon_right = actuator_commands[CA_ELEVON_RIGHT]
        ctrl_msg.elevon_left = actuator_commands[CA_ELEVON_LEFT]
        return ctrl_msg

def nonlinear_ctrl_optimization(x, thrust_torque_desired, v_body, airspeed, prev_solution):
    K_Tau = CA.K_Tau
    K_delta = CA.K_delta(airspeed)
    x_des = CA.actuators_desired
    # x_des = np.array([1, 1, 1, 0, 0, 1, 1])
    # K2 = np.eye(7) * np.array([[5, 5, 5, 0, 0, 0.0, 0.0]]).T * 1e-7 * airspeed**2
    # K2 = np.eye(7) * np.array([[1, 1, 10, 3, 3, 0.0, 0.0]]).T * 1e-6 * airspeed**2
    x_0, z_0, x_1, z_1 = calc_rectangular_rotor_components(x)

    # compute the thrust/torque and its derivative with respect to the change of throttle
    Va_rear = (np.array([0.0, 0.0, -1.0]).T @ v_body)[0]
    Va_right = (np.array([x_0, 0.0, -z_0]).T @ v_body)[0]
    Va_left  = (np.array([x_1, 0.0, -z_1]).T @ v_body)[0]
    thrust, torque, thrust_der, torque_der = \
        rotor_thrust_torque_der(x[CA_ROTOR_RIGHT:CA_ROTOR_REAR + 1], 
        [Va_right, Va_left, Va_rear])

    # Compute elevon forces
    elevon_force_coefs = calc_elevon_force(v_body, airspeed)

    thrust_torque_achieved = _calc_thrust_torque_achieved(
        x, thrust, torque, elevon_force_coefs, airspeed, v_body)
    thrust_torque_diff = thrust_torque_desired - thrust_torque_achieved
    diff_norm = 0.5 * thrust_torque_diff.T @ K_Tau @ thrust_torque_diff \
        + .5 * (x - x_des).T @ K_delta @ (x - x_des)

    thrust_torque_der = calc_thrust_torque_achieved_der(
        x, thrust, torque, thrust_der, torque_der, elevon_force_coefs, airspeed)
    diff_norm_der = -thrust_torque_der @ K_Tau @ thrust_torque_diff \
        + K_delta @ (x - x_des)
    
    return diff_norm, diff_norm_der

# Returns the thrust and torque achieved by certain deltas
# x is the proposed setpoint for each of the actuators
# thrust is a three-dimensional vector of thrust achieved by each of the three rotors
# torque is a three-dimensional vector representing the torque caused by each of the 
# rotors
def _calc_thrust_torque_achieved(x, thrust, torque, elevon_force_coefs, airspeed, v_body):
    x_0, z_0, x_1, z_1 = calc_rectangular_rotor_components(x)
    q0 = VTOL.right_rotor_pos.reshape(-1)
    q1 = VTOL.left_rotor_pos.reshape(-1)
    q2 = VTOL.rear_rotor_pos.reshape(-1)
    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing
    alpha = np.arctan(v_body[2]/v_body[0])[0] if v_body[0] != 0 else 0.0

    T_x =  (thrust[0] * x_0) + (thrust[1] * x_1) + \
        elevon_force_coefs[0] * (x[CA_ELEVON_RIGHT] + x[CA_ELEVON_LEFT])
    T_z = -(thrust[0] * z_0) - (thrust[1] * z_1) - thrust[2] + \
        elevon_force_coefs[1] * (x[CA_ELEVON_RIGHT] + x[CA_ELEVON_LEFT])
    Tau_x = - (torque[0] * x_0) - (thrust[0] * q0[1] * z_0) \
            - (torque[1] * x_1) - (thrust[1] * q1[1] * z_1) \
            - (thrust[2] * q2[1]) \
            - Gamma * VTOL.b * VTOL.C_ell_delta_a * x[CA_ELEVON_RIGHT] \
            + Gamma * VTOL.b * VTOL.C_ell_delta_a * x[CA_ELEVON_LEFT]
    Tau_y = thrust[0] * (q0[2] * x_0 + q0[0] * z_0) + \
            thrust[1] * (q1[2] * x_1 + q1[0] * z_1) + \
            thrust[2] * q2[0] + \
            Gamma * VTOL.c * VTOL.C_m_delta_e * x[CA_ELEVON_RIGHT] + \
            Gamma * VTOL.c * VTOL.C_m_delta_e * x[CA_ELEVON_LEFT] #+ \
            # Gamma * VTOL.c * VTOL.C_m_alpha * alpha
    Tau_z = - (thrust[0] * q0[1] * x_0) + (torque[0] * z_0) \
            - (thrust[1] * q1[1] * x_1) + (torque[1] * z_1) \
            + torque[2]

    return np.array([T_x, T_z, Tau_x, Tau_y, Tau_z]).T

# Calculates the gradient of the thrust and torque achieved
def calc_thrust_torque_achieved_der(
    x, thrust, torque, thrust_der, torque_der, elevon_force_coefs, airspeed):

    x_0, z_0, x_1, z_1 = calc_rectangular_rotor_components(x)

    q0 = VTOL.right_rotor_pos.reshape(-1)
    q1 = VTOL.left_rotor_pos.reshape(-1)
    q2 = VTOL.rear_rotor_pos.reshape(-1)

    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing

    T_x_der =  [thrust_der[0] * x_0,
                thrust_der[1] * x_1,
                0,
                -thrust[0] * z_0,
                -thrust[1] * z_1,
                elevon_force_coefs[0],
                elevon_force_coefs[0]]
    T_z_der = [-thrust_der[0] * z_0,
                -thrust_der[1] * z_1,
                -thrust_der[2],
                -thrust[0] * x_0,
                -thrust[1] * x_1,
                elevon_force_coefs[1],
                elevon_force_coefs[1]]
    Tau_x_der = [-(torque_der[0] * x_0) - (thrust_der[0] * q0[1] * z_0),
                    -(torque_der[1] * x_1) - (thrust_der[1] * q1[1] * z_1),
                    - (thrust_der[2] * q2[1]),
                    torque[0] * z_0 - thrust[0] * q0[1] * x_0,
                    torque[1] * z_1 - thrust[1] * q1[1] * x_1,
                    - Gamma * VTOL.b * VTOL.C_ell_delta_a,
                    + Gamma * VTOL.b * VTOL.C_ell_delta_a]
    Tau_y_der = [thrust_der[0] * (q0[2] * x_0 + q0[0] * z_0),
                    thrust_der[1] * (q1[2] * x_1 + q1[0] * z_1),
                    thrust_der[2] * q2[0],
                    thrust[0] * (-q0[2] * z_0 + q0[0] * x_0),
                    thrust[1] * (-q1[2] * z_1 + q1[0] * x_1),
                    Gamma * VTOL.c * VTOL.C_m_delta_e,
                    Gamma * VTOL.c * VTOL.C_m_delta_e]
    Tau_z_der = [-(thrust_der[0] * q0[1] * x_0) + (torque_der[0] * z_0),
                    -(thrust_der[1] * q1[1] * x_1) + (torque_der[1] * z_1),
                    torque_der[2],
                    thrust[0] * q0[1] * z_0 + torque[0] * x_0,
                    thrust[1] * q1[1] * z_1 + torque[1] * x_1,
                    0,
                    0]
    return np.array([T_x_der, T_z_der, Tau_x_der, Tau_y_der, Tau_z_der]).T

def rotor_thrust_torque_der(delta, Va):
    thrust = list()
    torque = list()
    thrust_der = list()
    torque_der = list()
    for i in range(3):
        # compute thrust and torque due to propeller  (See addendum by McLain)
        # grab motor/prop params
        if i == CA_ROTOR_REAR:
            C_Q0 = VTOL.C_Q0_rear
            C_Q1 = VTOL.C_Q1_rear
            C_T0 = VTOL.C_T0_rear
            C_Q2 = VTOL.C_Q2_rear
            C_T1 = VTOL.C_T1_rear
            C_T2 = VTOL.C_T2_rear
            D_prop = VTOL.D_prop_rear
            KQ = VTOL.KQ_rear
            R_motor = VTOL.R_motor_rear
            i0 = VTOL.i0_rear
        else:
            C_Q0 = VTOL.C_Q0_front
            C_Q1 = VTOL.C_Q1_front
            C_Q2 = VTOL.C_Q2_front
            C_T0 = VTOL.C_T0_front
            C_T1 = VTOL.C_T1_front
            C_T2 = VTOL.C_T2_front
            D_prop = VTOL.D_prop_front
            KQ = VTOL.KQ_front
            R_motor = VTOL.R_motor_front
            i0 = VTOL.i0_front
        # map delta_t throttle command(0 to 1) into motor input voltage
        V_in = VTOL.V_max * delta[i]
        V_in_der = VTOL.V_max
        # Quadratic formula to solve for motor speed
        a = C_Q0 * VTOL.rho * np.power(D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (C_Q1 * VTOL.rho * np.power(D_prop, 4)
            / (2.*np.pi)) * Va[i] + KQ**2/R_motor
        c = C_Q2 * VTOL.rho * np.power(D_prop, 3) \
            * (Va[i])**2 - (KQ / R_motor) * V_in + KQ * i0
        c_der = (KQ / R_motor) * V_in_der
        # Consider only positive root
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        Omega_op_der = c_der / np.sqrt(b**2 - 4*a*c)
        # compute advance ratio
        J_op = 2 * np.pi * Va[i] / (Omega_op * D_prop)
        J_op_der = -2 * np.pi * Va[i] * Omega_op_der / (Omega_op**2 * D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
        C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
        C_T_der = 2 * C_T2 * J_op * J_op_der + C_T1 * J_op_der
        C_Q_der = 2 * C_Q2 * J_op * J_op_der + C_Q1 * J_op_der
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        T_p = VTOL.rho * n**2 * np.power(D_prop, 4) * C_T
        Q_p = VTOL.rho * n**2 * np.power(D_prop, 5) * C_Q
        T_p_der = VTOL.rho * Omega_op * Omega_op_der * np.power(D_prop, 4) * C_T / (2 * np.pi**2) + \
            VTOL.rho * Omega_op**2 * np.power(D_prop, 4) * C_T_der / (2 * np.pi)**2
        Q_p_der = VTOL.rho * Omega_op * Omega_op_der * np.power(D_prop, 5) * C_Q / (2 * np.pi**2) + \
            VTOL.rho * Omega_op**2 * np.power(D_prop, 5) * C_Q_der / (2 * np.pi)**2
        # Flip moment sign for 1 (left front rotor)
        if i == 1:
            Q_p *= -1
            Q_p_der *= -1

        thrust.append(T_p)
        torque.append(Q_p)
        thrust_der.append(T_p_der)
        torque_der.append(Q_p_der)

    return thrust, torque, thrust_der, torque_der

def calc_elevon_force(v_body, airspeed):
    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing
    elevon_lift_coef = Gamma * VTOL.C_L_delta_e
    elevon_drag_coef = Gamma * VTOL.C_D_delta_e
    if v_body[0] != 0:
        alpha = np.arctan2(v_body[2], v_body[0]).item(0)
    else:
        alpha = 0
    elevon_force_coefs = [-np.cos(alpha) * elevon_drag_coef + np.sin(alpha) * elevon_lift_coef,
                          -np.sin(alpha) * elevon_drag_coef - np.cos(alpha) * elevon_lift_coef]
    return elevon_force_coefs

def calc_rectangular_rotor_components(x):
    x_0 = np.cos(x[CA_SERVO_RIGHT])
    z_0 = np.sin(x[CA_SERVO_RIGHT])
    x_1 = np.cos(x[CA_SERVO_LEFT])
    z_1 = np.sin(x[CA_SERVO_LEFT])
    return x_0, z_0, x_1, z_1

def calc_thrust_torque_achieved(x, v_body, airspeed):
    x_0, z_0, x_1, z_1 = calc_rectangular_rotor_components(x)

    Va_rear = (np.array([0.0, 0.0, -1.0]).T @ v_body)[0]
    Va_right = (np.array([x_0, 0.0, -z_0]).T @ v_body)[0]
    Va_left  = (np.array([x_1, 0.0, -z_1]).T @ v_body)[0]
    thrust, torque, thrust_der, torque_der = \
        rotor_thrust_torque_der(x[CA_ROTOR_RIGHT:CA_ROTOR_REAR + 1], 
        [Va_right, Va_left, Va_rear])

    elevon_force_coefs = calc_elevon_force(v_body, airspeed)

    return _calc_thrust_torque_achieved(x, thrust, torque, elevon_force_coefs, airspeed, v_body)
