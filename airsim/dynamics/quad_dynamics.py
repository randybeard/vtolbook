"""
quadDynamics 
    - this file implements the dynamic equations of motion for quadrotor
    - use unit quaternion for the attitude state
"""
import numpy as np

# load message types
from message_types.msg_state import MsgState

from tools.rotations import rotation_to_quaternion, quaternion_to_rotation
from tools.rotations import hat, quat_hat


class QuadDynamics:
    def __init__(self, Ts, QUAD):
        self.QUAD = QUAD
        self._ts_simulation = Ts
        # set initial states based on parameter file

        quat0 = rotation_to_quaternion(self.QUAD.rot0)

        self._state = np.array([[self.QUAD.pos0.item(0)],    # (0)
                               [self.QUAD.pos0.item(1)],     # (1)
                               [self.QUAD.pos0.item(2)],     # (2)
                               [self.QUAD.vel0.item(0)],     # (3)
                               [self.QUAD.vel0.item(1)],     # (4)
                               [self.QUAD.vel0.item(2)],     # (5)
                               [quat0.item(0)],         # (6)
                               [quat0.item(1)],         # (7)
                               [quat0.item(2)],         # (8)
                               [quat0.item(3)],         # (9)
                               [self.QUAD.omega0.item(0)],   # (10)
                               [self.QUAD.omega0.item(1)],   # (11)
                               [self.QUAD.omega0.item(2)]])  # (12)

        # initialize true_state message
        self.true_state = MsgState()
        self._update_true_state()

    ###################################
    # public functions
    def update(self, delta):
        """
            Integrate the differential equations defining dynamics, update sensors
            delta is the control inputs
            Ts is the time step between function calls.
        """

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._f(self._state, delta)
        k2 = self._f(self._state + time_step/2.*k1, delta)
        k3 = self._f(self._state + time_step/2.*k2, delta)
        k4 = self._f(self._state + time_step*k3, delta)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        normQuat = np.linalg.norm(self._state[6:10])
        self._state[6][0] /= normQuat
        self._state[7][0] /= normQuat
        self._state[8][0] /= normQuat
        self._state[9][0] /= normQuat

        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _f(self, state, delta):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        
        # pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:]

        R = quaternion_to_rotation(quat)
        e3 = np.array([[0.], [0.], [1.]])

        pos_dot = R@vel
        force = np.array([[0.], [0.], [-delta.force]])
        vel_dot = self.QUAD.gravity * e3 + R@force/self.QUAD.mass

        quat_dot = 0.5 * quat_hat(omega) @ quat
        omega_dot = self.QUAD.Jinv @ (-hat(omega) @ self.QUAD.J @ omega + delta.torque)
        x_dot = np.concatenate((pos_dot, vel_dot, quat_dot, omega_dot), axis=0)
        return x_dot

    def _update_true_state(self):
        # update the class structure for the true state:
        self.true_state.pos = self._state[0:3]
        self.true_state.vel = self._state[3:6]
        self.true_state.rot = quaternion_to_rotation(self._state[6:10])
        self.true_state.omega = self._state[10:]
