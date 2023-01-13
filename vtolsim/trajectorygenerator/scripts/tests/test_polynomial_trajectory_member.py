#! /usr/bin/env python3

import sys

import unittest
import numpy as np
import numpy.testing as nptst
import sympy as sp

sys.path.append('..')
from basis.standard_basis_member import StandardBasisMember
from polynomial_trajectory_member import PolynomialTrajectoryMember


class TestPolynomialTrajectoryMember(unittest.TestCase):

    def setUp(self):
        self.basis_vector_l4_d1 = [
            StandardBasisMember(1),
            StandardBasisMember(1),
            StandardBasisMember(1),
            StandardBasisMember(1),
        ]

        self.basis_vector_l5_d4 = [
            StandardBasisMember(0),
            StandardBasisMember(1),
            StandardBasisMember(2),
            StandardBasisMember(3),
            StandardBasisMember(4),
        ]

    def test_init_1(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*4),
            self.basis_vector_l4_d1,
            np.ones(5))

        self.assertEqual(PTM.num_bases, 4)
        self.assertEqual(PTM.num_segments, 5)
        self.assertIsInstance(PTM.num_bases, int)
        self.assertIsInstance(PTM.num_segments, int)

        nptst.assert_array_equal(
            PTM.coefficients,
            np.ones(5*4))

    def test_init_2(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.ones(7))

        self.assertEqual(PTM.num_bases, 5)
        self.assertEqual(PTM.num_segments, 7)
        self.assertIsInstance(PTM.num_bases, int)
        self.assertIsInstance(PTM.num_segments, int)

        nptst.assert_array_equal(
            PTM.coefficients,
            np.ones(5*7))

    def test_separate_tau_1(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.ones(7))

        idx, s = PTM._separate_tau(1.0)
        self.assertEqual(idx, 1)
        self.assertIsInstance(idx, int)

        self.assertEqual(s, 0.0)
        self.assertIsInstance(s, float)

    def test_separate_tau_15(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.ones(7))

        idx, s = PTM._separate_tau(3.5772)
        self.assertEqual(idx, 3)
        self.assertIsInstance(idx, int)

        self.assertAlmostEqual(s, 0.5772)
        self.assertIsInstance(s, float)

    def test_separate_tau_0(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.ones(7))

        idx, s = PTM._separate_tau(0)
        self.assertEqual(idx, 0)
        self.assertIsInstance(idx, int)

        self.assertEqual(s, 0.0)
        self.assertIsInstance(s, float)

    def test_separate_tau_7(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.ones(7))

        idx, s = PTM._separate_tau(7.0)
        self.assertEqual(idx, 6)
        self.assertIsInstance(idx, int)

        self.assertEqual(s, 1.0)
        self.assertIsInstance(s, float)

    def test_separate_tau_highlow(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.ones(7))

        with self.assertRaises(ValueError):
            _, _ = PTM._separate_tau(7.5)

        with self.assertRaises(ValueError):
            _, _ = PTM._separate_tau(17)

        with self.assertRaises(ValueError):
            _, _ = PTM._separate_tau(-.5)

    def test_eval_l5_d4(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.ones(7))

        res = PTM.eval(0.0, 0)
        self.assertEqual(res, 1.0)

        res = PTM.eval(1.0, 0)
        self.assertEqual(res, 1.0)

        res = PTM.eval(6.0, 0)
        self.assertEqual(res, 1.0)

        res = PTM.eval(3.5, 0)
        self.assertEqual(
            res,
            np.sum([
                .5**0 / np.math.factorial(0.),
                .5**1 / np.math.factorial(1.),
                .5**2 / np.math.factorial(2.),
                .5**3 / np.math.factorial(3.),
                .5**4 / np.math.factorial(4.),
                ]))

        res = PTM.eval(7.0, 0)
        self.assertEqual(
            res,
            np.sum([
                1.**0 / np.math.factorial(0.),
                1.**1 / np.math.factorial(1.),
                1.**2 / np.math.factorial(2.),
                1.**3 / np.math.factorial(3.),
                1.**4 / np.math.factorial(4.),
                ]))

    def test_separate_tau_seg_times_1(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.arange(1., 8.))

        self.assertEqual(PTM.total_time, 7+6+5+4.+3.+2.+1.)

        idx, s = PTM._separate_tau(.5)
        self.assertEqual(idx, 0)
        self.assertIsInstance(idx, int)

        self.assertEqual(s, .5)
        self.assertIsInstance(s, float)

    def test_separate_tau_seg_times_2(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.arange(1., 8.))

        idx, s = PTM._separate_tau(1.5)
        self.assertEqual(idx, 1)
        self.assertIsInstance(idx, int)

        self.assertEqual(s, .25)
        self.assertIsInstance(s, float)

    def test_separate_tau_seg_times_end(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.arange(1., 8.))

        idx, s = PTM._separate_tau(28.0)
        self.assertEqual(idx, 6)
        self.assertIsInstance(idx, int)

        self.assertEqual(s, 1.)
        self.assertIsInstance(s, float)

    def test_separate_tau_seg_times_near_end(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.arange(1., 8.))

        idx, s = PTM._separate_tau(27.5)
        self.assertEqual(idx, 6)
        self.assertIsInstance(idx, int)

        self.assertEqual(s, 6.5/7.0)
        self.assertIsInstance(s, float)

    def test_eval_seg_times(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.arange(1., 8.))

        res = PTM.eval(1.0, 0)
        self.assertEqual(res, 1.0)

        res = PTM.eval(1.0, 1)
        self.assertEqual(res, .5)

        res = PTM.eval(1.0, 2)
        self.assertEqual(res, .25)

    def test_eval_seg_times_near_end(self):
        PTM = PolynomialTrajectoryMember(
            np.ones(5*7),
            self.basis_vector_l5_d4,
            np.arange(1., 8.))

        res = PTM.eval(21.0, 0)
        self.assertEqual(res, 1.0)

        res = PTM.eval(21.0, 1)
        self.assertEqual(res, 1./7.)

        res = PTM.eval(21.0, 2)
        self.assertEqual(res, 1./(7.**2))
    

if __name__ == "__main__":
    unittest.main()
