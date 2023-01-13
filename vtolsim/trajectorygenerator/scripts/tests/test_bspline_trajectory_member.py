#! /usr/bin/env python3

import sys

import unittest
import numpy as np
import numpy.testing as nptst
import sympy as sp

sys.path.append('..')
from bspline_trajectory_member import BSplineTrajectoryMember


class TestBSplineTrajectoryMember(unittest.TestCase):

    def setUp(self):
        pass

    def test_init_1(self):
        BTM = BSplineTrajectoryMember(
            np.arange(0, 5, 1),
            3
        )

        self.assertEqual(BTM.degree, 3)
        self.assertEqual(len(BTM.derivative_splines), 4)
        self.assertEqual(BTM.total_time, 6.0)

    def test_setup_uniform_knot_sequence(self):
        ks = BSplineTrajectoryMember._setup_uniform_knot_sequence(4, 11)

        self.assertEqual(len(ks), 4 + 11 + 1 + 4 + 1)
        self.assertEqual(ks[0], 0)
        self.assertEqual(ks[5], 1)
        self.assertEqual(ks[-1], 11 + 1)

    def test_setup_pinned_control_points(self):
        first = 0
        last = 7
        step = 1
        degree = 3

        cp = BSplineTrajectoryMember._setup_pinned_control_points(
            np.arange(first, last, step),
            degree
        )

        self.assertEqual(len(cp), degree*2 + len(np.arange(first, last, step)))
        self.assertEqual(cp[0], first)
        self.assertEqual(cp[-1], last-step)

    def test_eval_1(self):
        BTM = BSplineTrajectoryMember(
            np.arange(0, 5, 1),
            3
        )

        with self.assertRaises(ValueError):
            # negative time
            _ = BTM.eval(-1.5, 0)

        with self.assertRaises(ValueError):
            # high derivative
            _ = BTM.eval(1.1, 27)

        self.assertAlmostEqual(BTM.eval(0.0, 0), 0.0)
        self.assertAlmostEqual(BTM.eval(0.0, 1), 0.0)
        self.assertAlmostEqual(BTM.eval(0.0, 2), 0.0)

        self.assertGreater(BTM.eval(1.0, 0), 0.0)
        self.assertGreater(BTM.eval(1.0, 1), 0.0)

        self.assertAlmostEqual(BTM.eval(29.77, 0), 4.0)
        self.assertAlmostEqual(BTM.eval(29.77, 1), 0.0)
        self.assertAlmostEqual(BTM.eval(27.77, 2), 0.0)

        self.assertIsInstance(BTM.eval(2.3, 0), float)
        self.assertIsInstance(BTM.eval(2.3, 1), float)
        self.assertIsInstance(BTM.eval(2.3, 2), float)


if __name__ == "__main__":
    unittest.main()
