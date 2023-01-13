"""
quadrotor_dynamics
    - this file implements the dynamic equations of motion
    - use unit quaternion for the attitude state

    - Update history:
        3/19/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

import parameters.quadrotor_parameters as QUAD
from tools.rotations import rotation_to_quaternion, quaternion_to_rotation
from tools.rotations import hat, quat_hat

# load message types
from message_types.msg_state import MsgState

class QuadrotorDynamics:
    def __init__(self, Ts):
        self.ts_simulation = Ts
        quat0 = rotation_to_quaternion(QUAD.rot0)
        # internal state (quaternion)
        self._state = np.array([
            [QUAD.pos0.item(0)],
            [QUAD.pos0.item(1)],
            [QUAD.pos0.item(2)],
            [QUAD.vel0.item(0)],
            [QUAD.vel0.item(1)],
            [QUAD.vel0.item(2)],
            [quat0.item(0)],
            [quat0.item(1)],
            [quat0.item(2)],
            [quat0.item(3)],
            [QUAD.omega0.item(0)],
            [QUAD.omega0.item(1)],
            [QUAD.omega0.item(2)],
        ])
        # visible state (rotation)
        self.true_state = MsgState()
        self.update_true_state()

    ###################################
    # public functions
    def update(self, delta):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        x1 = self.f(self._state, delta)
        x2 = self.f(self._state + time_step/2.*x1, delta)
        x3 = self.f(self._state + time_step/2.*x2, delta)
        x4 = self.f(self._state + time_step*x3, delta)
        self._state = self._state + time_step/6 * (x1 + 2*x2 + 2*x3 + x4)
        # normalize the quaternion
        normQuat = np.linalg.norm(self._state[6:10])
        self._state[6][0] /= normQuat
        self._state[7][0] /= normQuat
        self._state[8][0] /= normQuat
        self._state[9][0] /= normQuat
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
        R = quaternion_to_rotation(quat)
        e3 = np.array([[0.], [0.], [1.]])
        # translational equations of motion
        pos_dot = quaternion_to_rotation(quat) @ vel
        force = np.array([[0.], [0.], [-delta.force]])
        vel_dot = QUAD.gravity * e3 + R @ force/QUAD.mass
        # rotational equations of motion
        quat_dot = 0.5 * quat_hat(omega) @ quat
        omega_dot = QUAD.Jinv @ (-hat(omega) @ QUAD.J @ omega + delta.torque)
        # collect the derivative of the states
        x_dot = np.concatenate((pos_dot, vel_dot, quat_dot, omega_dot), axis=0)
        return x_dot

    def update_true_state(self):
        # update the state message:
        self.true_state.pos = self._state[0:3]
        self.true_state.vel = self._state[3:6]
        self.true_state.rot = quaternion_to_rotation(self._state[6:10])
        self.true_state.omega = self._state[10:13]