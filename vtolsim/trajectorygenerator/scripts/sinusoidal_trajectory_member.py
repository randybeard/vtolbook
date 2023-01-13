#!/usr/bin/env python3
# Trajectory Member for the sinusoidal trajectory type

import numpy as np
from trajectory_member import TrajectoryMember


class SinusoidalTrajectoryMember(TrajectoryMember):
    def __init__(self, offset, scale, period, phase):
        """
        Create a trajectory member P, whose 0th derivative is
        P = offset + range*sin((2*pi*/period)*tau + phase)
        """

        self.offset = offset
        self.scale = scale

        if period != 0.0:
            self.freq = 2.0*np.pi/period
        else:
            self.freq = 0.0

        self.phase = phase

    def eval(self, tau, k):
        """
        kth derivative wrt to tau, at tau
        """

        P_dk = self.scale * self.freq**k \
            * np.sin(self.freq*tau + self.phase + k*np.pi/2.0)

        if k == 0:
            P_dk += self.offset

        return P_dk
