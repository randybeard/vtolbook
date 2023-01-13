#!/usr/bin/env python3
# Trajectory Member for the sinusoidal trajectory type

import numpy as np
from trajectory_member import TrajectoryMember


class LinearTrajectoryMember(TrajectoryMember):
    def __init__(self, offset, slope):
        """
        Create a trajectory member P, whose 0th derivative is
        P = offset + slope*tau
        """

        self.offset = offset
        self.slope = slope

    def eval(self, tau, k):
        """
        kth derivative wrt to tau, at tau
        """
        P_dk = 0
        if k == 0:
            P_dk = self.offset + self.slope*tau
        elif k == 1:
            P_dk = self.slope
        else:
            P_dk = 0.0

        return P_dk
