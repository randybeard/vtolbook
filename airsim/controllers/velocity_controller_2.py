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

        # self.kR = np.diag([1.5,1.5,.5])
        # self.Komega = 20.*np.eye(3)

        self.kx = 2  # .4
        self.kv = 5 # .7
        self.kR = 1.5 # .1
        # self.kR = np.diag([1.5,1.5,0])
        self.kOmega = 1.5 # .1    
        # self.kOmega = np.diag([1.5,1.5,0])

    def update(self, trajectory, state):
        ex = 0
        ev = state.vel - trajectory.vel

        e3 = np.array([[0.],[0.],[1.]])

        # Get desired attitude
        thrustComponent = -self.kx*ex - self.kv*ev - self.QUAD.mass*self.QUAD.gravity*e3
        kd = -thrustComponent / np.linalg.norm(thrustComponent)
        # b2 = hat(b3)@b1
        # Rd = np.concatenate((b1,b2,b3)) # Not orthonormal??
        sd = np.array([[np.cos(trajectory.heading)], [np.sin(trajectory.heading)], [0.]])
        jd = hat(kd) @ sd
        jd = jd / np.linalg.norm(jd)
        id = hat(jd) @ kd
        Rd = np.concatenate((id, jd, kd), axis=1)

        R_d = concatenate((id, jd, kd), axis=1)
        R_d_dot = self.Rdot_der.update(R_d)
        
        eR = 0.5 * vee(Rd.T@state.rot - state.rot.T@Rd)

        omegad = vee(0.5*(Rd.T@R_d_dot - R_d_dot.T@Rd)) # jake's code line 73 is different TODO
        omegadDot = self.omegadot_der.update(omegad)

        eOmega = state.omega - state.rot.T@Rd@omegad

        T = saturate(np.dot(-(thrustComponent).T, state.rot[:,2])[0], 0, self.QUAD.Tmax)
        tau = -self.kR*eR - self.kOmega * eOmega + hat(state.omega) @ self.QUAD.J@state.omega \
            - self.QUAD.J @ (hat(state.omega)@state.rot.T@Rd@omegad - state.rot.T@Rd@omegadDot)
        
        delta = MsgDelta(force=T, torque=tau)
        commanded_state = MsgState(pos=trajectory.pos,
                                   vel=trajectory.vel,
                                   rot=Rd,
                                   omega=state.rot.T@Rd@omegad)
        return delta, commanded_state
        
        

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output



    