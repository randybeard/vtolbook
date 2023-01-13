#!/usr/bin/env python3
# Trajectory Member for the sinusoidal trajectory type

import numpy as np
from trajectory_member import TrajectoryMember


class QuadraticTrajectoryMember(TrajectoryMember):
    def __init__(self, a, b, c):
        """
        Create a trajectory member P, whose 0th derivative is
        P = .5*a*tau^2 + b*tau + c
        """

        self.a = a
        self.b = b
        self.c = c

    def eval(self, tau, k):
        """
        kth derivative wrt to tau, at tau
        """
        P_dk = 0
        if k == 0:
            P_dk = .5*self.a*tau**2 + self.b*tau + self.c
        elif k == 1:
            P_dk = self.a*tau + self.b
        elif k == 2:
            P_dk = self.a
        else:
            P_dk = 0.0

        return P_dk
