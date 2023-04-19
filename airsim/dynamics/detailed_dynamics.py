"""
quadDynamics 
    - this file implements the dynamic equations of motion for quadrotor
    - use unit quaternion for the attitude state
"""
import numpy as np
if __name__ == "__main__":
    import pathlib
    import sys

    # this line gets the parent folder of the parent folder of the current file
    # i.e. vtolbook/airsim/. If your sim file is in the airsim folder, you don't need this
    directory = pathlib.Path(__file__).parent.parent
    # this is so we can have the base folder of the project (vtolbook/airsim) in the path
    sys.path.append(str(directory))

# load message types
from message_types.msg_state import MsgState

from tools.rotations import rotation_to_quaternion, quaternion_to_rotation
from tools.rotations import hat, quat_hat


class QuadDynamics:
    def __init__(self, Ts, QUAD):
        self.QUAD = QUAD
        self._ts_simulation = Ts
        # set initial states based on parameter file
        self.e3 = self.QUAD.e3

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
                               [self.QUAD.omega0.item(2)],   # (12)
                            #    [0.],[0.],[1.]
                               ])  

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

        # norm_n = np.linalg.norm(self._state[13:])
        # self._state[13][0] /= norm_n
        # self._state[14][0] /= norm_n
        # self._state[15][0] /= norm_n

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
        omega = state[10:13]
        n = state[13:]
        p = omega.item(0)
        q = omega.item(1)
        r = omega.item(2)

        w1 = -np.sqrt(np.abs(delta.forces.item(0))/self.QUAD.k_f)
        w2 = np.sqrt(delta.forces.item(1)/self.QUAD.k_f)
        w3 = -np.sqrt(np.abs(delta.forces.item(2))/self.QUAD.k_f)
        w4 = np.sqrt(delta.forces.item(3)/self.QUAD.k_f)

        R = quaternion_to_rotation(quat)

        e3 = np.array([[0., 0., 1.]]).T

        pos_dot = vel
        force = np.sum(delta.forces)
        vel_dot = -e3 * self.QUAD.gravity + R@e3*force/self.QUAD.mass

        quat_dot = 0.5 * quat_hat(omega) @ quat
        
        #omega_dot
        p_d = (self.QUAD.k_f*(w2**2-w4**2)*self.QUAD.l - (self.QUAD.Izz_t - self.QUAD.Ixx_t)*q*r - self.QUAD.Izz_p*q*(w1+w2+w3+w4))/self.QUAD.Ixx_b
        q_d = (self.QUAD.k_f*(w3**2-w1**2)*self.QUAD.l + (self.QUAD.Izz_t - self.QUAD.Ixx_t)*p*r + self.QUAD.Izz_p*p*(w1+w2+w3+w4))/self.QUAD.Ixx_b
        r_d = (-self.QUAD.gamma*r+self.QUAD.k_t*self.QUAD.k_f*(w1**2-w2**2+w3**2-w4**2)) / self.QUAD.Izz_b

        # n_dot = -hat(omega)@n

        x_dot = np.concatenate((pos_dot, vel_dot, quat_dot, np.array([[p_d, q_d, r_d]]).T), axis=0)
        return x_dot

    def _update_true_state(self):
        # update the class structure for the true state:
        self.true_state.pos = self._state[0:3]
        self.true_state.vel = self._state[3:6]
        self.true_state.rot = quaternion_to_rotation(self._state[6:10])
        self.true_state.omega = self._state[10:13]
        # self.true_state.body_axis = self._state[13:]


if __name__ == "__main__":
    import parameters.detalied_quadrotor_parameters as QUAD
    from message_types.msg_motor_commands import MsgDelta

    dynamics = QuadDynamics(0.05, QUAD)

    quat0 = rotation_to_quaternion(QUAD.rot0)
    
    state = np.array([[QUAD.pos0.item(0)],    # (0)
                      [QUAD.pos0.item(2)],     # (2)
                      [QUAD.pos0.item(1)],     # (1)
                      [QUAD.vel0.item(0)],     # (3)
                      [QUAD.vel0.item(1)],     # (4)
                      [QUAD.vel0.item(2)],     # (5)
                      [quat0.item(0)],         # (6)
                      [quat0.item(1)],         # (7)
                      [quat0.item(2)],         # (8)
                      [quat0.item(3)],         # (9)
                      [QUAD.wb.item(0)],   # (10)
                      [QUAD.wb.item(1)],   # (11)
                      [QUAD.wb.item(2)],   # (12)
                    #   [QUAD.n.item(0)],
                    #   [QUAD.n.item(1)],
                    #   [QUAD.n.item(2)]
                      ])
    delta = MsgDelta(np.array([[QUAD.f1], [QUAD.f2], [QUAD.f3], [QUAD.f4]]))

    x_dot = dynamics._f(state, delta)
    print(x_dot)