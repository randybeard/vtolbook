"""
differential_flatness.py
    - Update history:
        3-JUL-2019 - J.B. Willis
"""
import sys
sys.path.append('..')

import numpy as np
from trajectory_planning.trajectory import Trajectory
from tools.rotations import Euler2Rotation, Euler2Quaternion
from tools.quaternions import q_rotate, q_conj
import parameters.convergence_parameters as VTOL
import parameters.differential_flatness_parameters as DF


class DifferentialFlatness:
    def __init__(self, traj):
        self.traj = traj
        self.near_s = -1
        self._clearInternal()

    def _clearInternal(self):
        # clear internal state
        # we keep track of these to reduce computation time
        self._desP     = None
        self._desPdot  = None
        self._desPddot = None
        self._desVbody = None
        self._desEuler = None
        self._desQuat  = None

    def desiredState_fromX(self, x):
        """
        Given a current state x, find the next desired state
        """
        cur_p = x[0:3]
        cur_v = x[3:6]
        
        if self.near_s == -1:
            self.near_s = self.traj.findNearestP(cur_p)
        else:
            self.near_s = self.traj.findNearestPSeeded(cur_p, self.near_s)

        nearest_state, des_input = self.desiredState(self.near_s)
        
        return nearest_state, des_input
        
    def desiredState(self, t):
        """ Given a time, get the desired state at that time """
        self._clearInternal() # start with a clean slate
        p = self.desiredPosition(t)
        q = self.desiredQuat(t)
        v = self.desiredVelocity(t)
        u = self.desiredInput(t)
        return np.concatenate((p, v, q), axis=0), u


    def desiredInput(self, t):
        F = self.desiredThrust(t)
        return np.array([[F.item(0), F.item(1), 0., 0., 0.]]).T


    def desiredPosition(self, t):
        if self._desP is None:
            self._desP = self.traj.getP(t)
        return  self._desP

    def desiredVelocity(self, t):
        
        if(self._desPdot is None):
            self._desPdot = self.traj.getPdot(t)
        v_inertial = self._desPdot

        if(self._desQuat is None):
            self._desQuat = self.desiredQuat(t)
        
        v_body = q_rotate(v_inertial, q_conj(self._desQuat))
        self._desVbody = v_body
        return v_body

    def desiredEuler(self, t):
        phi_d   = self._desiredPhi(t)
        theta_d = self._desiredTheta(t)
        psi_d   = self._desiredPsi(t)
        self._desEuler = np.array([[phi_d, theta_d, psi_d]]).T
        return self._desEuler

    def _desiredPhi(self, t):
        return 0.0 # level flight is preferred

    def _desiredTheta(self, t):
        # assume wind is zero and level flight,
        # so gamma=gamma_a=0 and theta=alpha
        return 0.0 

    def _desiredAlpha(self, t):
        if self._desPdot is None:
            self._desPdot = self.traj.getPdot(t)

        # u and w defined in the vehicle frame to prevent
        # circular dependency on body frame angles 
        u = np.linalg.norm(self._desPdot[0:2])
        w = np.linalg.norm(self._desPdot.item(2))

        mix_u = (u*(DF.k_u)**(-u**2))
        mix_u = mix_u/DF.mix_u_max # scale by the max value

        mix_w = ((DF.k_w)**(-w**2))

        return DF.alpha_max*mix_u*mix_w
        
    def _desiredPsi(self, t):
        pdot = self.traj.getPdot(t)
        psi = np.arctan2(pdot.item(1), pdot.item(0))
        return np.squeeze(psi)

    def _curvature(self, t):
        T = self.traj.getPdot(t)
        Tdot = self.traj.getPddot(t)
        s = np.linalg.norm(T)
        T_norm = T/s
        Tdot_norm = Tdot/s

        k = _project(T_norm)@Tdot_norm
        return k

    def desiredQuat(self, t):
        if self._desQuat is not None:
            return self._desQuat

        if self._desEuler is None:
            self._desEuler = self.desiredEuler(t)
        desEuler = self._desEuler
        self._desQuat = Euler2Quaternion(desEuler.item(0), desEuler.item(1), desEuler.item(2))
        return self._desQuat 

    def desiredRotation(self, t):
        if self._desEuler is None:
            self._desEuler = self.desiredEuler(t)
        desEuler = self._desEuler
        desRot = Euler2Rotation(desEuler.item(0), desEuler.item(1), desEuler.item(2))
        return desRot
    
    def desiredThrust(self, t):
        if self._desPdot is None:
            self._desPdot = self.traj.getPdot(t)

        v_horiz = np.linalg.norm(self._desPdot[0:2])
        v_vert = -(self._desPdot.item(2))
        v_norm = np.linalg.norm(self._desPdot)

        gamma = np.arctan2(v_vert, v_horiz)
       
        if v_norm > .001:
            ax = (v_horiz/v_norm)*.4
            az = (v_vert/v_norm)*.8
            if az < 0:
                az = -az
                ax = 0.2*ax
        else:
            ax = 0.0
            # az = 0.1
            az = VTOL.mass * VTOL.gravity

        return np.array([[ax, az]]).T

    def desiredRotdot(self, t):
        delta = .01
        desRot_t = self.desiredRotation(t)
        desRot_tm1 = self.desiredRotation(t-delta)
        diff = np.subtract(desRot_t, desRot_tm1)
        Rotdot = diff/delta
        return Rotdot

    def desiredRates(self, t):
        desRot_t = self.desiredRotation(t)
        Rotdot_t = self.desiredRotdot(t)
        omega_x = desRot_t @ Rotdot_t

        return _deSkew(omega_x)

def _project(v):
    """ project a three-vector onto a plane """
    v_norm = v/np.linalg.norm(v)
    return (np.eye(3) - v_norm @ v_norm.T)


def _deSkew(A):
    """ convert a skew symmetric matrix to a 3-vector by averaging the symmetric values """
    a1 = (A[2,1] - A[1, 2])/2
    a2 = (A[0,2] - A[2, 0])/2
    a3 = (A[1,0] - A[0, 1])/2

    return np.array([[a1, a2, a3]]).T
