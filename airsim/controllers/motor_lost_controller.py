import numpy as np
from numpy import concatenate
from scipy.linalg import solve_continuous_are, norm
from tools.dirty_derivative import DirtyDerivative
from tools.rotations import quaternion_to_rotation, vee, hat, logR, leftJacobianInv
from message_types.msg_state import MsgState
from message_types.msg_motor_commands import MsgDelta
from message_types.msg_autopilot import MsgAutopilot


class Autopilot:
    def __init__(self, ts_control, QUAD):
        self.QUAD = QUAD
        self.Ts = ts_control
        self.pos_error_der = DirtyDerivative(self.Ts, 5*self.Ts)
        self.e3 = self.QUAD.e3

        Q = np.diag([0.,0.,1000.,2.])
        R = np.diag([0.75])
        P = solve_continuous_are(self.QUAD.A, self.QUAD.B, Q, R)
        self.K = np.linalg.inv(R) @ self.QUAD.B.T @ P
        

    def update(self, trajectory, state):
        #outer loop: get the desired acceleration of the quad
        pos_error = state.pos - trajectory.pos
        pos_error_d = self.pos_error_der.update(pos_error)

        # altitude_error = pos_error.item(2)
        # altitude_error_d = pos_error_d.item(2)
        # des_altitude_acc = -2*self.QUAD.eta*self.QUAD.wn_altitude*altitude_error_d - self.QUAD.wn_altitude**2*altitude_error

        des_acc = -2*self.QUAD.eta*self.QUAD.wn_heading*pos_error_d - self.QUAD.wn_heading**2*pos_error
        # des_acc[2,:] = des_altitude_acc

        #inner loop: get the motor commands

        #solve for the desired normal vector and total force
        nf = self.QUAD.mass*state.rot.T @ (des_acc - (-self.e3)*self.QUAD.gravity)/self.QUAD.nz
        f_t = np.linalg.norm(nf)
        n_des = nf / f_t
        # n_des = des_acc - (-self.e3)*self.QUAD.gravity
        # n_des /= np.linalg.norm(n_des)

        #get the state we care about
        p = state.omega.item(0)
        q = state.omega.item(1)
        nx = n_des.item(0)
        ny = n_des.item(1)
        e = np.array([[p, q, nx, ny]]).T

        #for two lost case, u = (f3-f3b) - (f1-f1b), or f3 - f1
        u = (-self.K@e)
        u1 = u.item(0)
        f3 = saturate((f_t + u1)/2, self.QUAD.Tmin, self.QUAD.Tmax)
        f1 = saturate((f_t - u1)/2, self.QUAD.Tmin, self.QUAD.Tmax)
        f2 = 0
        f4 = 0

        delta = MsgDelta(np.array([[f1, f2, f3, f4]]).T)
        return delta, state

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output



    