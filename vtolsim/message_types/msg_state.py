"""
msgState 
    - messages type for state, that will be passed between blocks in the architecture
    
part of mavPySim 
    - Beard & McLain, PUP, 2012
    - Update history:  
        1/9/2019 - RWB
"""

import numpy as np
import sys
sys.path.append('..')
from tools.rotations import Euler2Rotation
class msgState:
    def __init__(self):
        self.pn = 0.      # inertial north position in meters
        self.pe = 0.      # inertial east position in meters
        self.h = 0.       # inertial altitude in meters
        self.phi = 0.     # roll angle in radians
        self.theta = 0.   # pitch angle in radians
        self.psi = 0.     # yaw angle in radians
        self.Va = 0.      # airspeed in meters/sec
        self.alpha = 0.   # angle of attack in radians
        self.beta = 0.    # sideslip angle in radians
        self.p = 0.       # roll rate in radians/sec
        self.q = 0.       # pitch rate in radians/sec
        self.r = 0.       # yaw rate in radians/sec
        self.Vg = 0.      # groundspeed in meters/sec
        self.gamma = 0.   # flight path angle in radians
        self.chi = 0.     # course angle in radians
        self.wn = 0.      # inertial windspeed in north direction in meters/sec
        self.we = 0.      # inertial windspeed in east direction in meters/sec
        self.bx = 0.      # gyro bias along roll axis in radians/sec
        self.by = 0.      # gyro bias along pitch axis in radians/sec
        self.bz = 0.      # gyro bias along yaw axis in radians/sec
        self.right_rotor = 0.  # angle of the right motor in radians
        self.left_rotor = 0.  # angle of the left motor in radians
        self.u = 0.       # inertial velocity resolved along body x-axis in m/s
        self.v = 0.       # inertial velocity resolved along body y-axis in m/s
        self.w = 0.       # inertial velocity resolved along body z-axis in m/s

    @property
    def position(self):
        return np.array([[self.pn, self.pe, -self.h]]).T

    @property
    def velocity(self):
        return np.array([[self.u, self.v, self.w]]).T

    @property
    def rot(self):
        return Euler2Rotation(self.phi, self.theta, self.psi)

    @property
    def omega(self):
        return np.array([[self.p, self.q, self.r]]).T