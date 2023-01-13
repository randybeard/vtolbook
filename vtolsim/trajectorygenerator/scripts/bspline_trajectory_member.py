#!/usr/bin/env python3
# Trajectory Member for the BSpline trajectory type

from os import stat
import numpy as np
from scipy.interpolate import BSpline
from trajectory_member import TrajectoryMember


class BSplineTrajectoryMember(TrajectoryMember):
    def __init__(self, path_points, degree, knot_sequence=None):
        """
        Create a BSpline trajectory member using path points as
        intermediate control points.
        When knot_sequence is None, uses a uniform sequence and
        assumes endpoints should be pinned.
        """
        self.degree = degree
        num_path_points = len(path_points)

        if knot_sequence is None:
            knot_sequence = self._setup_uniform_knot_sequence(
                self.degree, num_path_points
            )
            control_points = self._setup_pinned_control_points(
                path_points, self.degree
            )
            self.total_time = float(num_path_points + 1)
        else:
            # otherwise, assume knot sequence and path points are
            # properly shaped for BSpline
            self.total_time = knot_sequence[-1]

        self.spline = BSpline(knot_sequence, control_points, degree)

        # get derivative splines up to degree - 1
        self.derivative_splines = [self.spline]
        for deriv_k in range(1, degree + 1):
            self.derivative_splines.append(
                self.spline.derivative(nu=deriv_k)
            )

    @staticmethod
    def _setup_uniform_knot_sequence(degree, num_path_points):
        # uniform knot sequence, endpoints pinned
        knot_sequence = np.concatenate([
            np.zeros(degree),
            np.arange(0, num_path_points + 1, 1.0),
            (num_path_points + 1) * np.ones(degree + 1)
            ])

        return knot_sequence

    @staticmethod
    def _setup_pinned_control_points(path_points, degree):
        control_points = np.concatenate([
            np.ones(degree-1)*path_points[0],
            path_points,
            np.ones(degree+1)*path_points[-1]
        ])

        return control_points

    def eval(self, tau, k):
        if tau < 0.0:
            raise ValueError(
                "Invalid nondimensional time tau = {}".format(tau))
        elif tau > self.total_time:
            tau = self.total_time

        if k > self.degree:
            raise ValueError(
                f"""Can't take derivative {k} which is higher than
                spline degree {self.degree}""")

        return self.derivative_splines[k]([tau])[0]
