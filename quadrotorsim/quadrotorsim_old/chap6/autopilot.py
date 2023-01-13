"""
autopilot block for quadrotorsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/3/2022 - RWB
"""
import sys
from numpy import array, zeros, eye, concatenate, diag, sin, cos, cross
from scipy.linalg import solve_continuous_are, norm
sys.path.append('..')
from tools.dirty_derivative import DirtyDerivative
from tools.rotations import vee, hat, logR, leftJacobianInv
import parameters.quadrotor_parameters as QUAD
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

# Jake Johnson's autopilot from 2022 L-CSS paper
class Autopilot:
    def __init__(self, ts_control):
        self.Ts = ts_control
        self.integrator_pos = zeros((3, 1))
        self.pos_error_delay_1 = zeros((3, 1))
        # compute lqr gains
        Id = eye(3)
        Zero = zeros((3,3))
        A = concatenate((
                concatenate((Zero, Id, Zero), axis=1),
                concatenate((Zero, Zero, Zero), axis=1),
                concatenate((Id, Zero, Zero), axis=1),
            ), axis=0)
        B = concatenate((Zero, Id, Zero), axis=0)
        qp = 10.0  # lqr turning parameters
        qd = 20.0
        qi = 0.1
        qf = 1.0
        Q = diag([qp, qp, qp, qd, qd, qd, qi, qi, qi])
        R = qf * diag([1., 1., 1.])
        P = solve_continuous_are(A, B, Q, R)
        self.K_lqr = (1/qf) * B.T @ P
        # instantiate dirty derivatives for R_d and omega_d
        self.Rdot = DirtyDerivative(Ts=self.Ts, tau=5*self.Ts)
        self.omegadot = DirtyDerivative(Ts=self.Ts, tau=5*self.Ts)
        # gains for Jake Johnson's controller
        self.Kr = 20*diag([1., 1., 1.])
        self.Komega = 20*diag([1., 1., 1.])


    def update(self, trajectory, state):
        # compute errors
        pos_error = state.pos - trajectory.pos
        vel_error = state.vel - trajectory.vel
        e3 = array([[0.], [0.], [1.]])
        # update the integrator
        self.integrator_pos = self.integrator_pos + self.Ts/2 * (pos_error + self.pos_error_delay_1)
        self.error_pos_delay_1 = pos_error
        # compute the desired force from LQR
        x_tilde = concatenate((pos_error, vel_error, self.integrator_pos), axis=0)
        f_d = trajectory.accel  \
              - QUAD.mass * QUAD.gravity * e3 \
              - self.K_lqr @ x_tilde
        thrust = saturate(norm(f_d), 0, QUAD.Tmax)
        #T = -f_d.T @ state.rot @ e3
        #thrust = saturate(T.item(0), 0, QUAD.Tmax)
        # compute the desired rotation matrix
        k_d = - f_d / (norm(f_d)+0.01)
        s_d = array([[cos(trajectory.heading)], [sin(trajectory.heading)], [0.]])
        j_d = cross(k_d.T, s_d.T).T
        j_d = j_d / norm(j_d)
        i_d = cross(j_d.T, k_d.T).T
        R_d = concatenate((i_d, j_d, k_d), axis=1)
        # compute derivative of R
        R_d_dot = self.Rdot.update(R_d)
        omega_d = vee(0.5*(R_d.T @ R_d_dot - R_d_dot.T @ R_d))
        omega_d_dot = self.omegadot.update(omega_d)
        r_tilde = vee(logR(state.rot.T @ R_d))
        omega_tilde = state.rot.T @ R_d @ omega_d - state.omega
        torque = hat(state.omega) @ QUAD.J @ state.omega \
                 + QUAD.J @ omega_d_dot \
                 + leftJacobianInv(r_tilde).T @ self.Kr @ r_tilde \
                 + self.Komega @ omega_tilde

        # construct output and commanded states
        delta = MsgDelta(force=thrust, torque=torque)
        commanded_state = MsgState(pos=trajectory.pos,
                                   vel=trajectory.vel,
                                   rot=R_d,
                                   omega=omega_d)
        return delta, commanded_state

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output
