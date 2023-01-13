#!/usr/bin/env python3
# Trajectory base class
# - Defines the interface for a trajectory
# - Provides functions for time-scaling, and for evaluating all
#   members of the trajectory at specified derivatives.

import numpy as np


class Trajectory:

    def __init__(self, members, time_scale=1.0):
        self.time_scale = time_scale
        self.Nr = len(members)
        self.members = members

    def tau(self, t):
        """
        Scale the time.
        """
        return self.time_scale * t

    def evalUpToKr(self, t, kr, ell=None):
        """
        Evaluate the trajectory and its derivatives up to and including kr
        at time t, returning them as an (Nr x kr) numpy array.

        If ell is a scalar, evaluate only the ellth member of the trajectory.
        If ell is an array, evaluate at the indices given by the array.
        """

        ell_vec, Nell = self._validate_ell(ell)

        kr += 1  # up to and including the derivative specified by kr

        Rflag = np.zeros((Nell, kr))
        for k in range(kr):
            Rflag[:, k] = self.eval(t, k, ell_vec)

        return Rflag

    def eval(self, t, k, ell=None):
        """
        Evaluate the trajectory and its kth derivative at time t

        If ell is a scalar, evaluate only the ellth member of the trajectory.
        If ell is an array, evaluate at the indices given by the array.
        """

        tau = self.tau(t)

        ell_vec, Nell = self._validate_ell(ell)

        R_k = np.zeros(Nell)
        for ell_i in ell_vec:
            Rell_k = self.members[ell_i].eval(tau, k)
            R_k[ell_i] = Rell_k

        # handle chain rule for P(tau(t))
        R_k *= self.time_scale**k

        return R_k

    def _validate_ell(self, ell):
        if ell is None:
            # evaluate all members of the trajectory
            Nell = self.Nr
            ell = np.arange(0, Nell, 1, dtype=int)
        elif isinstance(ell, int):
            # evaluate a single member of the trajectory
            Nell = 1
            ell = np.int64([ell])
        elif isinstance(ell, (list, np.ndarray)):
            # evaluate multiple members of the trajectory
            Nell = len(ell)
            ell = np.int64(ell)
        else:
            raise TypeError

        return ell, Nell
