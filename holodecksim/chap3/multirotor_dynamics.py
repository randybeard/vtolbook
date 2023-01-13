"""
multirotor_dynamics
    - this file implements the dynamic equations of motion
    - use unit quaternion for the attitude state

    - Update history:
        3/19/2020 - RWB
        6/23/2020
"""
import sys
sys.path.append('..')
import numpy as np

import parameters.multirotor_parameters as SYS
from tools.rotations import rotation_to_quaternion, quaternion_to_rotation
from tools.rotations import hat, quat_hat

# load message types
from message_types.msg_state import msgState

class multirotorDynamics:
    def __init__(self, Ts):
        self.ts_simulation = Ts
        quat0 = rotation_to_quaternion(SYS.rot0)
        # internal state (quaternion)
        self.internal_state = np.array([
            [SYS.pos0.item(0)],
            [SYS.pos0.item(1)],
            [SYS.pos0.item(2)],
            [SYS.vel0.item(0)],
            [SYS.vel0.item(1)],
            [SYS.vel0.item(2)],
            [quat0.item(0)],
            [quat0.item(1)],
            [quat0.item(2)],
            [quat0.item(3)],
            [SYS.omega0.item(0)],
            [SYS.omega0.item(1)],
            [SYS.omega0.item(2)],
            [SYS.gimbal0.item(0)],
            [SYS.gimbal0.item(1)],
            [SYS.gimbal0.item(2)],
        ])
        # visible state (rotation)
        self.true_state = msgState()
        self.update_true_state()

    ###################################
    # public functions
    def update(self, delta):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        x1 = self.f(self.internal_state, delta)
        x2 = self.f(self.internal_state + time_step/2.*x1, delta)
        x3 = self.f(self.internal_state + time_step/2.*x2, delta)
        x4 = self.f(self.internal_state + time_step*x3, delta)
        self.internal_state += time_step/6 * (x1 + 2*x2 + 2*x3 + x4)
        # normalize the quaternion
        normQuat = np.linalg.norm(self.internal_state[6:10])
        self.internal_state[6][0] /= normQuat
        self.internal_state[7][0] /= normQuat
        self.internal_state[8][0] /= normQuat
        self.internal_state[9][0] /= normQuat
        # update the message class for the true state
        self.update_true_state()

    def f(self, state, delta):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        gimbal = state[13:16]
        R = quaternion_to_rotation(quat)
        e3 = np.array([[0.], [0.], [1.]])
        # translational equations of motion
        pos_dot = vel
        D = np.array([[SYS.gravity * SYS.Cd, 0, 0], [0, SYS.gravity * SYS.Cd, 0], [0, 0, 0]])
        vel_dot = SYS.gravity * e3 - (delta.force / SYS.mass) * R @ e3 + R @ D @ R.T @ vel
        # rotational equations of motion
        quat_dot = 0.5 * quat_hat(omega) @ quat
        omega_dot = SYS.Jinv @ (-hat(omega) @ SYS.J @ omega + delta.torque)
        # gimbal dynamics (first order dynamics)
        gimbal_dot = SYS.k_gimbal * (delta.gimbal_input - gimbal)
        # collect the derivative of the states
        x_dot = np.vstack((pos_dot, vel_dot, quat_dot, omega_dot, gimbal_dot))
        return x_dot

    def update_true_state(self):
        # update the state message:
        self.true_state.pos = self.internal_state[0:3]
        self.true_state.vel = self.internal_state[3:6]
        self.true_state.rot = quaternion_to_rotation(self.internal_state[6:10])
        self.true_state.omega = self.internal_state[10:13]
        self.true_state.gimbal = self.internal_state[13:16]