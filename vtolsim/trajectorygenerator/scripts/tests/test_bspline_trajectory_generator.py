#! /usr/bin/env python3

import sys

import unittest
import numpy as np
import numpy.testing as nptst
import sympy as sp

sys.path.append('..')
from bspline_trajectory_generator import BSplineTrajectoryGenerator


class TestBSplineTrajectoryGenerator(unittest.TestCase):

    def test_init_1(self):
        BTG = BSplineTrajectoryGenerator()

        traj = BTG.generate_trajectory(
            np.row_stack([
                np.arange(0, 5, 1),
                np.arange(3, 8, 1),
                np.arange(-2, 3, 1)
            ]),
            3
        )

        self.assertEqual(traj.Nr, 3)
        self.assertEqual(traj.time_scale, 1.0)

        traj_eval_0 = traj.evalUpToKr(0.0, 2)
        self.assertEqual(traj_eval_0.shape, (3, 3))
        nptst.assert_array_almost_equal(
            traj_eval_0[:, 0], np.array([0., 3., -2.]))
        nptst.assert_array_almost_equal(
            traj_eval_0[:, 1], np.array([0., 0., 0.]))



if __name__ == "__main__":
    unittest.main()
