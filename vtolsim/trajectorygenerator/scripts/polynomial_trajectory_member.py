#!/usr/bin/env python3

import numpy as np

from trajectory_member import TrajectoryMember
from basis.basis_member import BasisMember


class PolynomialTrajectoryMember(TrajectoryMember):
    """
    A piecewise polynomial with uniform knot spacing.
    """

    def __init__(self, coefficients, basis_vector, segment_times):
        """
        Create a single piecewise polynomial
        coefficients - a length m*n vector of polynomial coefficients
        basis_vector - a length n vector consisting of basis functions
        """
        self.num_bases = len(basis_vector)
        self.num_segments = int(len(coefficients)/self.num_bases)

        self.coefficients = coefficients
        self.basis_vector = basis_vector
        self.segment_times = segment_times
        self.total_time = np.sum(self.segment_times)

    def eval(self, tau, k):
        end_of_trajectory = False
        if tau < 0.0:
            raise ValueError(
                "Invalid nondimensional time tau = {}".format(tau))
        if tau > self.total_time:
            tau = self.total_time
            end_of_trajectory = True

        # handle end of trajectory by returning endpoint position
        # with zero derivatives
        if end_of_trajectory and k > 0:
            return 0.0

        seg_idx, s = self._separate_tau(tau)

        # get coefficients for this segment
        coef_seg_idx = seg_idx * self.num_bases
        seg_coefficients = \
            self.coefficients[coef_seg_idx:(coef_seg_idx + self.num_bases)]

        # get segment time for this segment
        seg_time = self.segment_times[seg_idx]

        # evaulate bases at s
        basis_dk_at_s = BasisMember.eval_basis_vector(self.basis_vector, s, k)

        P_dk_tau = (1/seg_time)**k * seg_coefficients @ basis_dk_at_s

        return P_dk_tau

    def _separate_tau(self, tau):
        if tau > self.total_time or tau < 0.0:
            raise ValueError(
                "Invalid nondimensional time tau = {}".format(tau))
        elif tau == self.total_time:
            idx = self.num_segments - 1
            s = 1.0
        else:
            idx = 0
            running_time = self.segment_times[idx]

            # move along segment times until we've passed tau
            while(tau >= running_time):
                idx += 1
                running_time += self.segment_times[idx]

            running_time -= self.segment_times[idx]

            s = (tau - running_time) / self.segment_times[idx]

        return idx, s
