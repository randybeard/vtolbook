import numpy as np
from numpy import concatenate
from scipy.linalg import solve_continuous_are, norm
from tools.dirty_derivative import DirtyDerivative
from tools.rotations import vee, hat, logR, leftJacobianInv
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot


class Autopilot:
    def __init__(self, ts_control, QUAD):
        self.QUAD = QUAD
        self.Ts = ts_control
        self.integrator_pos = np.zeros((3,1))
        self.pos_error_delay = np.zeros((3,1))

        Id = np.eye(3)
        Zero = np.zeros((3,3))
        A = concatenate((
                concatenate((Zero, Id, Zero), axis=1),
                concatenate((Zero, Zero, Zero), axis=1),
                concatenate((Id, Zero, Zero), axis=1),
        ), axis=0)
        B = concatenate((Zero, Id, Zero), axis=0)
        qp = 10.0
        qd = 20.0
        qi = 0.1
        qf = 1.0
        Q = np.diag([qp, qp, qp, qd, qd, qd, qi, qi, qi])
        R = qf * np.eye(3)
        P = solve_continuous_are(A, B, Q, R)
        self.K_lqr = (1/qf) * B.T @ P

        self.Rdot_der = DirtyDerivative(Ts=self.Ts, tau=5*self.Ts)
        self.omegadot_der = DirtyDerivative(Ts=self.Ts, tau=5*self.Ts)

        self.Kr = 20.*np.eye(3)
        self.Komega = 20.*np.eye(3)


    def update(self, trajectory, state):
        pos_error = state.pos - trajectory.pos
        vel_error = state.vel - trajectory.vel
        e3 = np.array([[0.],[0.],[1.]])

        self.integrator_pos = self.integrator_pos + self.Ts/2*(pos_error + self.pos_error_delay)
        self.pos_error_delay = pos_error

        x_e = concatenate((pos_error, vel_error, self.integrator_pos), axis=0)
        f_d = self.QUAD.mass * (trajectory.accel - self.QUAD.gravity * e3) - self.K_lqr @ x_e
        thrust = saturate(norm(f_d), 0, self.QUAD.Tmax)

        k_d = -f_d / (norm(f_d) + 0.01)
        s_d = np.array([[np.cos(trajectory.heading), np.sin(trajectory.heading), 0.]]).T
        j_d = np.cross(k_d.T, s_d.T).T
        j_d = j_d / norm(j_d)
        i_d = np.cross(j_d.T, k_d.T).T
        R_d = concatenate((i_d, j_d, k_d), axis=1)
        R_d_dot = self.Rdot_der.update(R_d)
        w_d = vee(0.5 * (R_d.T @ R_d_dot - R_d_dot.T @ R_d))
        w_d_dot = self.omegadot_der.update(w_d)

        r_tilde = vee(logR(state.rot.T @ R_d))
        w_tilde = state.rot.T @ R_d @ w_d - state.omega

        torque = hat(state.omega)@self.QUAD.J@state.omega + self.QUAD.J@state.rot.T@R_d@w_d_dot \
                   + leftJacobianInv(r_tilde).T@self.Kr@r_tilde + self.Komega@w_tilde

        delta = MsgDelta(force=thrust, torque=torque)
        commanded_state = MsgState(pos=trajectory.pos,
                                   vel=trajectory.vel,
                                   rot=R_d,
                                   omega=w_d)
        return delta, commanded_state

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output



    