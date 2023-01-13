#! /usr/bin/env python3

import sys
sys.path.append('..')

import unittest
import numpy as np
import numpy.testing as nptst
import sympy as sp

from basis.standard_basis_member import StandardBasisMember
from polynomial_trajectory_member import PolynomialTrajectoryMember
from polynomial_trajectory_generator import PolynomialTrajectoryGenerator


class TestPolynomialTrajectoryGenerator(unittest.TestCase):

    def test_init(self):
        # tests the initialization and computation of the spline matrix

        PTG = PolynomialTrajectoryGenerator(
            6,  # num segments
            7,  # num bases
            StandardBasisMember,
            4,  # continuity derivative
            4,  # smoothness derivative
            [
                {'deriv': 0, 'knot_list': [0, 1, 2, 3, 4, 5, 6]},
                {'deriv': 1, 'knot_list': [0, 1, 2]},
            ]
        )

        c = 7 + 3 + (6 - 1)*4

        self.assertEqual(PTG.spline_matrix.shape[0], 6*7)
        self.assertEqual(PTG.spline_matrix.shape[1], c)

    def test_generate_constraint_vector_a_bar(self):
        kcv = [
            {'deriv': 1, 'value_list': [5, 2, 7, 9, 3]},
            {'deriv': 2, 'value_list': [11, 12, 13]},
            {'deriv': 0, 'value_list': [21, 22, 23, 24, 25]}
        ]

        res = PolynomialTrajectoryGenerator._generate_constraint_vector_a_bar(
            kcv
        )

        self.assertEqual(res.shape[0], 13)
        self.assertEqual(res[0], 21)
        self.assertEqual(res[-1], 13)

    def test_generate_Phi_i_vector(self):
        res = PolynomialTrajectoryGenerator._generate_Phi_i_vector(
            5,
            7,
            StandardBasisMember,
            0,
            1.0,
            0
        )

        self.assertEqual(len(res), 5*7)

        self.assertEqual(1.0, res[0])
        self.assertEqual(1.0, res[1])
        self.assertEqual(1./2., res[2])
        self.assertEqual(0.0, res[-1])
        self.assertEqual(0.0, res[-8])

    def test_generate_Phi_i_vector_2(self):
        res = PolynomialTrajectoryGenerator._generate_Phi_i_vector(
            5,
            7,
            StandardBasisMember,
            4,
            1.0,
            0)

        self.assertEqual(len(res), 5*7)

        self.assertEqual(0.0, res[0])
        self.assertEqual(1./np.math.factorial(6), res[-1])
        self.assertEqual(1./np.math.factorial(5), res[-2])
        self.assertEqual(0.0, res[-8])

    def test_generate_Phi_i_vector_3(self):
        with self.assertRaises(ValueError):
            _ = PolynomialTrajectoryGenerator._generate_Phi_i_vector(
                5,
                7,
                StandardBasisMember,
                5,
                1.0,
                0)

    def test_generate_knot_constraint_matrix_Dka_1(self):
        res = (PolynomialTrajectoryGenerator
               ._generate_knot_constraint_matrix_Dka(
                    7,
                    5,
                    StandardBasisMember,
                    0,
                    [0, 1, 5, 7]
                )
               )

        self.assertEqual(res.shape[0], 4)
        self.assertEqual(res.shape[1], 5*7)

        # should have same result for all knots but last
        self.assertEqual(np.sum(res[0, :]), np.sum(res[1, :]))
        self.assertEqual(np.sum(res[1, :]), np.sum(res[2, :]))
        self.assertNotEqual(np.sum(res[2, :]), np.sum(res[3, :]))

    def test_generate_knot_constraint_matrix_Dka_2(self):
        res = (PolynomialTrajectoryGenerator
               ._generate_knot_constraint_matrix_Dka(
                    7,
                    5,
                    StandardBasisMember,
                    3,
                    [0, 1, 5, 7]
                )
               )

        self.assertEqual(res.shape[0], 4)
        self.assertEqual(res.shape[1], 5*7)

        # should have same result for all knots but last
        self.assertEqual(np.sum(res[0, :]), np.sum(res[1, :]))
        self.assertEqual(np.sum(res[1, :]), np.sum(res[2, :]))
        self.assertNotEqual(np.sum(res[2, :]), np.sum(res[3, :]))

    def test_generate_derivative_continuity_matrix_Dc_1(self):
        res = (PolynomialTrajectoryGenerator
               ._generate_continuity_derivative_matrix_Dc_k(
                    9,
                    7,
                    StandardBasisMember,
                    3
                )
               )

        self.assertEqual(res.shape[0], 8)
        self.assertEqual(res.shape[1], 9*7)

        # should have same result for all knots
        self.assertAlmostEqual(np.sum(res[0, :]), np.sum(res[1, :]))
        self.assertAlmostEqual(np.sum(res[1, :]), np.sum(res[2, :]))
        self.assertAlmostEqual(np.sum(res[2, :]), np.sum(res[3, :]))
        self.assertAlmostEqual(np.sum(res[6, :]), np.sum(res[7, :]))

    def test_generate_derivative_continuity_matrix_Dc_2(self):
        res = (PolynomialTrajectoryGenerator
               ._generate_continuity_derivative_matrix_Dc_k(
                    9,
                    7,
                    StandardBasisMember,
                    0
                )
               )

        self.assertEqual(res.shape[0], 8)
        self.assertEqual(res.shape[1], 9*7)

        # should have same result for all knots
        self.assertAlmostEqual(np.sum(res[0, :]), np.sum(res[1, :]))
        self.assertAlmostEqual(np.sum(res[1, :]), np.sum(res[2, :]))
        self.assertAlmostEqual(np.sum(res[2, :]), np.sum(res[3, :]))
        self.assertAlmostEqual(np.sum(res[6, :]), np.sum(res[7, :]))

    def test_generate_derivative_matrix_D(self):
        res = (PolynomialTrajectoryGenerator
               ._generate_derivative_matrix_D(
                    6,
                    7,
                    StandardBasisMember,
                    4,
                    [
                        {'deriv': 0, 'knot_list': [0, 1, 2, 3, 4, 5, 6]},
                        {'deriv': 1, 'knot_list': [0, 1, 2]},
                    ]
               ))

        c = 7 + 3 + (6 - 1)*4

        self.assertEqual(res.shape[0], c)
        self.assertEqual(res.shape[1], 6*7)

        self.assertEqual(res[0, 0], 1.0)
        self.assertEqual(res[1, 7], 1.0)
        self.assertEqual(res[2, 14], 1.0)
        self.assertEqual(res[7, 0], 0.0)
        self.assertEqual(res[7, 1], 1.0)

    def test_compute_min_derivative_gramian(self):
        nptst.assert_array_almost_equal(
            PolynomialTrajectoryGenerator._compute_min_derivative_gramian_W(
                9, StandardBasisMember, 4
            ),
            self._compute_min_derivative_gramian_symbolic(
                9, 4
            )
        )

        nptst.assert_array_almost_equal(
            PolynomialTrajectoryGenerator._compute_min_derivative_gramian_W(
                9, StandardBasisMember, 0
            ),
            self._compute_min_derivative_gramian_symbolic(
                9, 0
            )
        )

        nptst.assert_array_almost_equal(
            PolynomialTrajectoryGenerator._compute_min_derivative_gramian_W(
                15, StandardBasisMember, 7
            ),
            self._compute_min_derivative_gramian_symbolic(
                15, 7
            )
        )

        nptst.assert_array_almost_equal(
            PolynomialTrajectoryGenerator._compute_min_derivative_gramian_W(
                15, StandardBasisMember, 4
            ),
            self._compute_min_derivative_gramian_symbolic(
                15, 4
            )
        )

    @staticmethod
    def _compute_min_derivative_gramian_symbolic(
            num_basis,
            k):
        """
        Compute the gramian symbolically, assuming the
        StandardBasisMember is being used
        """

        s = sp.symbols('s')
        basis_vector = np.array([s**j/np.math.factorial(j)
                                for j in range(num_basis)])
        bv_diff = sp.diff(basis_vector, (s, k))

        outer_mat = sp.Array(np.outer(bv_diff, bv_diff))
        gram_mat = outer_mat.applyfunc(
            lambda v: sp.integrate(v, (s, 0, 1)).evalf()
        )
        gram_np = np.array(gram_mat.tolist()).astype(np.float64)
        return gram_np


if __name__ == "__main__":
    unittest.main()
