#!/usr/bin/python3

import numpy as np
import math
import sys
from scipy.fftpack import diff
from scipy.optimize import minimize
sys.path.append('..')


from dynamics.vtol_dynamics import vtolDynamics
import parameters.convergence_parameters as VTOL
import parameters.landing_parameters as P
from geometric_control.geometric_controller import GeometricController
import parameters.simulation_parameters as SIM
import tools.rotations as rot



class MinimumTimeLandingCalculator2D():

    def __init__(self, V_trans=P.V_trans, gamma=P.gamma, V_f=P.V_f, p_f=P.p_f):

        self.V_trans = V_trans
        self.gamma = gamma
        self.V_f=-P.V_f

        dynamics = vtolDynamics()
        Va_eq = -P.V_trans*P.gain #/ 2 # airspeed used to determine maximum thrust
        T_front_max, _ = dynamics._motor_thrust_torque(Va_eq, 1.0, False)
        T_rear_max, _ = dynamics._motor_thrust_torque(Va_eq, 1.0, True)
        self.T_max = 2*T_front_max + T_rear_max

        num_samples = 100
        self.T_max = 0
        print('T_max calc')
        for i in range(num_samples):
            Va = -V_trans * i / (num_samples - 1)
            T_front_max, _ = dynamics._motor_thrust_torque(Va, 1.0, False)
            T_rear_max, _ = dynamics._motor_thrust_torque(Va, 1.0, True)
            self.T_max += 2*T_front_max + T_rear_max
        self.T_max /= num_samples
        print('T_max final:')
        print(self.T_max)

        self._compute_p0()
        self.debug=0

        # # Time at which the vehicle starts transition to air brake
        # self.t_1 = self.t_f - P.Va * VTOL.mass / self.T_max
        # self.p_1 = P.Va*self.t_1
        # self.dist_p1_pf = .5 * P.Va**2*VTOL.mass / self.T_max
        # self.p_f = self.p_1 + self.dist_p1_pf

    def _compute_p0(self):
        a1 = -1/(VTOL.mass)*self.T_max / (self.V_trans*np.sin(self.gamma)-self.V_f)
        a2 = 1/(VTOL.mass)*self.T_max / (self.V_trans*np.cos(self.gamma))
        a3 = -VTOL.gravity / (self.V_trans*np.sin(self.gamma)-self.V_f)
        self.theta = np.arccos(a3/np.sqrt(a1**2+a2**2)) - np.arctan(a2/a1)
        self.t_trans = (self.V_trans*np.cos(self.gamma)) / (1/(VTOL.mass)*self.T_max*np.sin(self.theta))
        self.p_trans = self.backward_position_function(self.t_trans)
        self.p_f = np.zeros((2, 1))

        print(f'pitch: {np.degrees(self.theta)}')
        print(f'end_time: {self.t_trans}')
        print(np.degrees(self.theta - self.gamma))
        
    def _reposition_start_at_origin(self):
        self.p_f -= self.p_trans
        self.p_trans -= self.p_trans

    def reposition_p0(self, new_p0):
        self._reposition_start_at_origin()
        self.p_trans += new_p0
        self.p_f += new_p0


    def backward_position_function(self, t):
        return 1/VTOL.mass \
            * np.array([[-self.T_max*np.sin(self.theta)],
                        [-self.T_max*np.cos(self.theta) + VTOL.mass*VTOL.gravity]]) * (t**2)/2 \
            + np.array([[0], [self.V_f]])*t

    def position_function(self, forward_t):
        if forward_t > self.t_trans:
            return self.p_f
        t = self.t_trans - forward_t
        return 1/VTOL.mass \
            * np.array([[-self.T_max*np.sin(self.theta)],
                        [-self.T_max*np.cos(self.theta) + VTOL.mass*VTOL.gravity]]) * (t**2)/2 \
            + np.array([[0], [self.V_f]])*t + self.p_f

    def velocity_function(self, forward_t):
        if forward_t > self.t_trans:
            return np.array([[0], [0]])
        t = self.t_trans - forward_t
        return -(1/VTOL.mass \
            * np.array([[-self.T_max*np.sin(self.theta)],
                        [-self.T_max*np.cos(self.theta) + VTOL.mass*VTOL.gravity]]) * t \
            + np.array([[0], [self.V_f]]))

    def acceleration_function(self, forward_t):
        if forward_t > self.t_trans:
            return np.array([[0], [0]])
        t = self.t_trans - forward_t
        return 1/VTOL.mass \
            * np.array([[-self.T_max*np.sin(self.theta)],
                        [-self.T_max*np.cos(self.theta) + VTOL.mass*VTOL.gravity]])

    def jerk_function(self, t):
        return np.array([[0], [0]])

    def eval_derivatives_up_to_K(self, K, t):
        if K < 0:
            return -1
        derivs = self.eval_derivative_at_t(0, t)
        for i in range(1, K+1):
            derivs = np.concatenate((derivs, self.eval_derivative_at_t(i, t)), axis=1)
        derivs = np.insert(derivs, 1, 0, axis=0)
        # derivs[0,0] = -derivs[0,0]
        # derivs[2,0] = -derivs[2,0]
        # derivs[0,:] = -derivs[0,:]
        # derivs[2,:] = -derivs[2,:]
        # print(derivs)
        # print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
        # if self.debug == 0:
        #     self.debug = 1
        # else:
        #     exit(0)
        return derivs

    def eval_derivative_at_t(self, deriv, t):
        if deriv == 0:
            return self.position_function(t)
        elif deriv == 1:
            return self.velocity_function(t)
        elif deriv == 2:
            return self.acceleration_function(t)
        else:
            return np.zeros((2,1))

    def get_xyz_points(self, theta, times):
        points = list()
        for t in times:
            p = self.position_function(t)
            points.append([p*np.cos(theta), 0, -p*np.sin(theta)])
        return points

    def write_params_to_file(self):
        with open('../parameters/landing_parameters_precomputed.py', 'w') as f:
            print(f'u0 = {self.V_trans * np.cos(self.theta - self.gamma)}', file=f)
            print(f'w0 = {self.V_trans * np.sin(self.theta - self.gamma)}', file=f)
            print(f'theta = {self.theta}', file=f)
            print(f'sim_time = {self.t_trans}', file=f)

class LandingEntrance():

    def __init__(self, height_to_transition):
        self.V_trans = P.V_trans
        self.gamma = P.gamma
        self.horizontal_flight_dist = P.horizontal_flight_dist
        self.height_to_transition = height_to_transition
        self.Va = P.V_trans
        self.r = P.circle_r

        assert self.r*(1-np.cos(self.gamma)) < self.height_to_transition
        self.t_start_downward_turn = self.horizontal_flight_dist / self.Va
        time_in_downard_turn = abs(self.gamma * self.r / self.Va)
        self.t_start_constant_descent = self.t_start_downward_turn + time_in_downard_turn
        self.t_trans = self.t_start_constant_descent \
            + (self.height_to_transition - self.r*(1-np.cos(self.gamma))) \
            / (-np.sin(self.gamma)*self.Va)

    def position_function(self, t):
        if t < self.t_start_downward_turn:
            return np.array([[self.Va*t],
                             [0]])
        elif t < self.t_start_constant_descent:
            shifted_t = t - self.t_start_downward_turn
            return np.array([[self.r*np.sin(self.Va*shifted_t/self.r) \
                                + self.horizontal_flight_dist],
                             [self.r - self.r*np.cos(self.Va*shifted_t/self.r)]])
        else:
            shifted_t = t - self.t_start_constant_descent
            p_end_downard_turn = \
                np.array([[self.horizontal_flight_dist+self.r*np.sin(-self.gamma)],
                          [self.r - self.r*np.cos(self.gamma)]])
            return np.array([[self.Va*np.cos(self.gamma)*(shifted_t)],
                             [-self.Va*np.sin(self.gamma)*(shifted_t)]]) \
                    + p_end_downard_turn

    def velocity_function(self, t):
        if t < self.t_start_downward_turn:
            return np.array([[self.Va],
                             [0]])
        elif t < self.t_start_constant_descent:
            shifted_t = t - self.t_start_downward_turn
            return np.array([[self.Va*np.cos(self.Va*shifted_t/self.r)],
                             [self.Va*np.sin(self.Va*shifted_t/self.r)]])
        else:
            return np.array([[self.Va*np.cos(self.gamma)],
                             [-self.Va*np.sin(self.gamma)]])

    def get_acceleration(self, t):
        if t < self.t_start_downward_turn or t > self.t_start_constant_descent:
            return np.zeros((2,1))
        else:
            shifted_t = t - self.t_start_downward_turn
            return np.array([[-(self.Va**2/self.r)*np.sin(self.Va*shifted_t/self.r)],
                             [(self.Va**2/self.r)*np.cos(self.Va*shifted_t/self.r)]])

    def eval_derivative_at_t(self, deriv, t):
        if deriv == 0:
            return self.position_function(t)
        elif deriv == 1:
            return self.velocity_function(t)
        elif deriv == 2:
            return self.get_acceleration(t)
        else:
            return np.zeros((2,1))

    def eval_derivatives_up_to_K(self, K, t):
        if K < 0:
            return -1
        derivs = self.eval_derivative_at_t(0, t)
        for i in range(1, K+1):
            derivs = np.concatenate((derivs, self.eval_derivative_at_t(i, t)), axis=1)
        derivs = np.insert(derivs, 1, 0, axis=0)
        return derivs

# class LandingEntrance():

#     def __init__(self, height_to_transition):
#         self.V_trans = P.V_trans
#         self.gamma = P.gamma
#         self.horizontal_flight_dist = P.horizontal_flight_dist
#         self.height_to_transition = height_to_transition
#         self.Va = P.V_trans
#         self.t_trans = self.height_to_transition / (-np.sin(self.gamma)*self.Va)

#     def position_function(self, t):
#         return np.array([[self.Va*np.cos(self.gamma)*(t)],
#                          [-self.Va*np.sin(self.gamma)*(t)]])

#     def velocity_function(self, t):
#         return np.array([[self.Va*np.cos(self.gamma)],
#                          [-self.Va*np.sin(self.gamma)]])

#     def eval_derivative_at_t(self, deriv, t):
#         if deriv == 0:
#             return self.position_function(t)
#         elif deriv == 1:
#             return self.velocity_function(t)
#         elif deriv == 2:
#             return np.zeros((2,1))
#         else:
#             return np.zeros((2,1))

#     def eval_derivatives_up_to_K(self, K, t):
#         if K < 0:
#             return -1
#         derivs = self.eval_derivative_at_t(0, t)
#         for i in range(1, K+1):
#             derivs = np.concatenate((derivs, self.eval_derivative_at_t(i, t)), axis=1)
#         derivs = np.insert(derivs, 1, 0, axis=0)
#         return derivs
    

class LandingRoutine():

    def __init__(self):
        self.height = P.height
        self.vert_land = MinimumTimeLandingCalculator2D()
        self.transition_z = self.vert_land.p_trans.item(1)
        self.landing_entrance = LandingEntrance(height_to_transition=(self.height + self.transition_z)) # flip sign from z to h
        self.vert_land.reposition_p0(
            self.landing_entrance.position_function(self.landing_entrance.t_trans))
        self.t_f = self.landing_entrance.t_trans + self.vert_land.t_trans
        print(self.landing_entrance.t_trans)
        print(self.landing_entrance.t_start_constant_descent)
        
    def eval_derivatives_up_to_K(self, K, t):
        if t < self.landing_entrance.t_trans:
            return self.landing_entrance.eval_derivatives_up_to_K(K, t)
        else:
            return self.vert_land.eval_derivatives_up_to_K(K, t - self.landing_entrance.t_trans)
    
    def position_function(self, t):
        if t < self.landing_entrance.t_trans:
            return self.landing_entrance.position_function(t)
        else:
            return self.vert_land.position_function(t - self.landing_entrance.t_trans)

    def velocity_function(self, t):
        if t < self.landing_entrance.t_trans:
            return self.landing_entrance.velocity_function(t)
        else:
            return self.vert_land.velocity_function(t - self.landing_entrance.t_trans)

    def write_params_to_file(self):
        geom_ctrl = GeometricController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)
        estimated_state=np.array([[0, 0, 0, 
            P.V_trans*np.cos(P.gamma), 0, -P.V_trans*np.sin(P.gamma),
            1, 0, 0, 0]]).T
        pos_derivatives_at_t = self.eval_derivatives_up_to_K(4, 0)
        yaw_derivatives_at_t = np.zeros((1, 5))
        traj_derivatives_at_t = np.concatenate((pos_derivatives_at_t, yaw_derivatives_at_t), axis=0)
        T, R_d, omega_c, pd_i, vd_b, omega_pip = geom_ctrl.update(estimated_state[0:10], traj_derivatives_at_t)

        with open('../parameters/landing_parameters_precomputed.py', 'w') as f:
            # print(f'u0 = {P.V_trans*np.cos(rot.Rotation2Euler(R_d)[1])}', file=f)
            # print(f'w0 = {-P.V_trans*np.sin(rot.Rotation2Euler(R_d)[1])}', file=f)
            print(f'u0 = {P.V_trans}', file=f)
            print(f'w0 = {0}', file=f)
            # print(f'theta = {rot.Rotation2Euler(R_d)[1]}', file=f)
            print(f'theta = 0.0', file=f)
            print(f'sim_time = {self.t_f + 1.5}', file=f)
            print(f'transition_z = {self.transition_z}', file=f)
            print(f'max_thrust = {self.vert_land.T_max}', file=f)
            # print(f'sim_time = {self.landing_entrance.t_trans + self.vert_land.t_trans}', file=f)

LR = LandingRoutine()
LR.write_params_to_file()

# M = MinimumTimeLandingCalculator2D()

# print(M.velocity_function(0))