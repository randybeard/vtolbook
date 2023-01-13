import re
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import linprog, minimize, NonlinearConstraint

import parameters.convergence_parameters as VTOL
from tools.msg_convert import *
from control_allocation.compute_linear_motor_model import compute_rotor_allocation_submatrix
from message_types.msg_controls import msgControls

class ControlAllocation:
    def __init__(self, servo0=np.pi/2, Va=0.0, gamma=0.0):

        self.rotor_allocation_matrix = compute_rotor_allocation_submatrix(servo0, Va_star=Va, gamma_star=gamma)
        self.previous_solution = None
        self.K = np.eye(5)
    
    def update(self, thrust, torques, airspeed):

        elevon_allocation_matrix = self._compute_elevon_allocation_submatrix(airspeed)

        augmented_allocation_matrix = np.concatenate([
            self.rotor_allocation_matrix, elevon_allocation_matrix],
            axis=1)

        thrust_torque_vector = np.concatenate([thrust, torques], axis=0).reshape(-1)

        rectangular_actuator_commands, _, _, _ = np.linalg.lstsq(augmented_allocation_matrix, thrust_torque_vector, rcond=None)

        return self._formulate_ctrl_msg(rectangular_actuator_commands)
    
    def update_lp(self, thrust, torques, airspeed):
        """
        Use a linear program to perform control allocation.
        """
        elevon_allocation_matrix = self._compute_elevon_allocation_submatrix(airspeed)

        augmented_allocation_matrix = np.concatenate([
            self.rotor_allocation_matrix, elevon_allocation_matrix],
            axis=1)

        thrust_torque_vector = np.concatenate([thrust, torques], axis=0).reshape(-1)

        # bounds = [(-.4226, 1.), (-.2, 1.), (-.4226, 1.), (-.2, 1.), (0., 1.), (-1., 1.), (-1., 1.)]
        bounds = [(-.4226,  1.0),
            (0.0,     1.0),
            (-.4225,  1.0),
            (0.0,     1.0),
            (0.0,     1.0),
            (-1.0,    1.0),
            (-1.0,    1.0)]
        cost = np.ones(7)
        cost[-2:] = 0.05 # less cost for using elevons
        lh_unbalanced = [[0.414, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         [1.0, 0.414, 0.0, 0.0, 0.0, 0.0, 0.0],
                         [0.0, 0.0, 0.414, 1.0, 0.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.414, 0.0, 0.0, 0.0],]
        rh_unbalanced = [[1.0],
                         [1.0],
                         [1.0],
                         [1.0]]

        # if self.previous_solution is None:
        if True:
            # res = linprog(cost, A_eq = augmented_allocation_matrix, b_eq = thrust_torque_vector, bounds=bounds)
            res = linprog(cost,
                        A_ub = lh_unbalanced,
                        b_ub = rh_unbalanced, 
                        A_eq = augmented_allocation_matrix,
                        b_eq = thrust_torque_vector, 
                        bounds=bounds)
        else:
            delta_step = 0.1
            A_deriv = np.concatenate([np.eye(7), -np.eye(7)], axis=0)
            b_deriv = np.concatenate([self.previous_solution + delta_step, -self.previous_solution + delta_step], axis=0)

            res = linprog(cost, A_ub=A_deriv, b_ub=b_deriv, A_eq = augmented_allocation_matrix, b_eq = thrust_torque_vector, bounds=bounds)

        return self._formulate_ctrl_msg(res.x)

    def update_opt(self, thrust, torques, airspeed):

        elevon_allocation_matrix = self._compute_elevon_allocation_submatrix(airspeed)

        augmented_allocation_matrix = np.concatenate([
            self.rotor_allocation_matrix, elevon_allocation_matrix],
            axis=1)

        thrust_torque_vector = np.concatenate([thrust, torques], axis=0).reshape(-1)

        rectangular_actuator_commands = \
            self._compute_nonlinear_optimization(augmented_allocation_matrix, thrust_torque_vector)

        return self._formulate_ctrl_msg(rectangular_actuator_commands)

    def _formulate_ctrl_msg(self, rectangular_actuator_commands):
        delta_r, theta_r = self._rectangular_to_polar_motor_commands(rectangular_actuator_commands)

        # saturate
        delta_r[0:3] = np.clip(delta_r[0:3], 0., 1.)
        theta_r[:] = np.clip(theta_r, VTOL.servo_min, VTOL.servo_max)
        rectangular_actuator_commands[-2:] = np.clip(rectangular_actuator_commands[-2:], VTOL.elevon_min, VTOL.elevon_max)

        ctrl_msg = msgControls()

        ctrl_msg.throttle_right = delta_r[0]
        ctrl_msg.throttle_left = delta_r[1]
        ctrl_msg.throttle_rear = delta_r[2]

        ctrl_msg.servo_right = theta_r[0]
        ctrl_msg.servo_left = theta_r[1]

        ctrl_msg.elevon_right = rectangular_actuator_commands[-2]
        ctrl_msg.elevon_left = rectangular_actuator_commands[-1]
        return ctrl_msg

    def _compute_nonlinear_optimization(self, allocation_matrix, thrust_torque_vector):

        def tiltrotor_constraints(x):
            # print(x)
            return [x[0] ** 2 + x[1] ** 2,
                    x[2] ** 2 + x[3] ** 2,
                    np.arctan2(x[1], x[0]),
                    np.arctan2(x[3], x[2])]

        def ctrl_optimization(x):
            thrust_torque_diff = thrust_torque_vector - allocation_matrix @ x
            return 0.5 * np.transpose(thrust_torque_diff) @ self.K @ thrust_torque_diff

        def ctrl_optimization_grad(x):
            thrust_torque_diff = thrust_torque_vector - allocation_matrix @ x
            return -np.transpose(allocation_matrix) @ self.K @ thrust_torque_diff

        if self.previous_solution is None:
            x0 = np.zeros(7)
        else:
            x0 = self.previous_solution
        bounds = [(-.4226,  1.0),
                  (0.0,     1.0),
                  (-.4226,  1.0),
                  (0.0,     1.0),
                  (0.0,     1.0),
                  (-1.0,    1.0),
                  (-1.0,    1.0)]
        constraints = [NonlinearConstraint(tiltrotor_constraints, lb=[0, 0, VTOL.servo_min, VTOL.servo_min],
            ub=[1.0, 1.0, VTOL.servo_max, VTOL.servo_max], keep_feasible=False)]
        
        res = minimize(ctrl_optimization, x0, bounds=bounds, constraints=constraints, jac=ctrl_optimization_grad)
        self.previous_solution = res.x
        return res.x

    @classmethod
    def _compute_elevon_allocation_submatrix(cls, airspeed):
        Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing

        # signs are different than in paper since dynamics uses different convention
        allocation_matrix = np.array([
            [0., 0.],
            [0., 0.],
            [-Gamma * VTOL.b * VTOL.C_ell_delta_a, Gamma * VTOL.b * VTOL.C_ell_delta_a],
            [Gamma * VTOL.c * VTOL.C_m_delta_e, Gamma * VTOL.c * VTOL.C_m_delta_e],
            [0., 0.]
        ])

        return allocation_matrix
        
    @classmethod
    def _rectangular_to_polar_motor_commands(cls, rectangular_vec):
        delta_r_1 = np.sqrt(rectangular_vec[0]**2 + rectangular_vec[1]**2)
        theta_r_1 = np.arctan2(rectangular_vec[1], rectangular_vec[0])

        delta_r_2 = np.sqrt(rectangular_vec[2]**2 + rectangular_vec[3]**2)
        theta_r_2 = np.arctan2(rectangular_vec[3], rectangular_vec[2])

        delta_r_3 = rectangular_vec[4]

        delta_r = np.array([delta_r_1, delta_r_2, delta_r_3])
        theta_r = np.array([theta_r_1, theta_r_2])

        return delta_r, theta_r

    # def update_combined_methods(self, thrust, torques, airspeed):
    #     elevon_allocation_matrix = self._compute_elevon_allocation_submatrix(airspeed)

    #     augmented_allocation_matrix = np.concatenate([
    #         self.rotor_allocation_matrix, elevon_allocation_matrix],
    #         axis=1)

    #     thrust_torque_vector = np.concatenate([thrust, torques], axis=0).reshape(-1)

    #     # Linear programming method setup
    #     bounds = [(-.4226,  1.0),
    #         (0.0,     1.0),
    #         (-.4226,  1.0),
    #         (0.0,     1.0),
    #         (0.0,     1.0),
    #         (-1.0,    1.0),
    #         (-1.0,    1.0)]
    #     cost = np.ones(7)
    #     cost[-2:] = 0.05 # less cost for using elevons
    #     cost[4] = 1.2
    #     lh_unbalanced = [[0.414, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #                      [1.0, 0.414, 0.0, 0.0, 0.0, 0.0, 0.0],
    #                      [0.0, 0.0, 0.414, 1.0, 0.0, 0.0, 0.0],
    #                      [0.0, 0.0, 1.0, 0.414, 0.0, 0.0, 0.0],]
    #     rh_unbalanced = [[1.0],
    #                      [1.0],
    #                      [1.0],
    #                      [1.0]]

    #     lp_method_solution = linprog(cost,
    #                             A_ub = lh_unbalanced,
    #                             b_ub = rh_unbalanced, 
    #                             A_eq = augmented_allocation_matrix,
    #                             b_eq = thrust_torque_vector, 
    #                             bounds=bounds)
        
    #     if lp_method_solution.success:
    #         rectangular_actuator_commands = lp_method_solution.x
    #         self.previous_solution = lp_method_solution.x
    #     else:
    #         rectangular_actuator_commands, _, _, _ = np.linalg.lstsq(augmented_allocation_matrix, thrust_torque_vector, rcond=None)

    #     return self._formulate_ctrl_msg(rectangular_actuator_commands)


