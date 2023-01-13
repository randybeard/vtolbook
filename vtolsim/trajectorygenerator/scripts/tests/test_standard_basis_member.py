#! /usr/bin/env python3

import sys
sys.path.append('..')

import unittest
import numpy as np
import numpy.testing as nptst
import sympy as sp

from basis.standard_basis_member import StandardBasisMember


class TestStandardBasisMember(unittest.TestCase):

    def test_degree_0(self):
        basis = StandardBasisMember(0)

        nptst.assert_equal(basis.evalDerivative(1.0, 0), 1.0)
        nptst.assert_equal(basis.evalDerivative(.4275, 0), 1.0)
        nptst.assert_equal(basis.evalDerivative(0.0, 0), 1.0)

        nptst.assert_equal(basis.evalDerivative(1.0, 1), 0.0)
        nptst.assert_equal(basis.evalDerivative(0.0, 1), 0.0)

        nptst.assert_equal(basis.evalDerivative(1.0, 7), 0.0)
        nptst.assert_equal(basis.evalDerivative(0.0, 7), 0.0)

    def test_degree_1(self):
        basis = StandardBasisMember(1)

        nptst.assert_equal(basis.evalDerivative(1.0, 0), 1.0)
        nptst.assert_equal(basis.evalDerivative(.4275, 0), .4275)
        nptst.assert_equal(basis.evalDerivative(0.0, 0), 0.0)

        nptst.assert_equal(basis.evalDerivative(1.0, 1), 1.0)
        nptst.assert_equal(basis.evalDerivative(0.0, 1), 1.0)

        nptst.assert_equal(basis.evalDerivative(1.0, 7), 0.0)
        nptst.assert_equal(basis.evalDerivative(0.0, 7), 0.0)

    def test_degree_5(self):
        basis = StandardBasisMember(5)

        nptst.assert_equal(
            basis.evalDerivative(1.0, 0),
            1.0 / np.math.factorial(5))
        nptst.assert_equal(
            basis.evalDerivative(.4275, 0),
            .4275**5 / np.math.factorial(5))
        nptst.assert_equal(basis.evalDerivative(0.0, 0), 0.0)

        nptst.assert_equal(
            basis.evalDerivative(1.0, 1),
            1.0 / np.math.factorial(4))
        nptst.assert_equal(
            basis.evalDerivative(.4275, 1),
            .4275**4 / np.math.factorial(4))
        nptst.assert_equal(basis.evalDerivative(0.0, 1), 0.0)

        nptst.assert_equal(
            basis.evalDerivative(1.0, 4),
            1.0 / np.math.factorial(1))
        nptst.assert_equal(
            basis.evalDerivative(.4275, 4),
            .4275 / np.math.factorial(1))
        nptst.assert_equal(basis.evalDerivative(0.0, 1), 0.0)

        nptst.assert_equal(basis.evalDerivative(1.0, 7), 0.0)
        nptst.assert_equal(basis.evalDerivative(0.0, 7), 0.0)

        # compare with symbolic result
        s = sp.symbols('s')
        basis_sp = s**5/np.math.factorial(5)
        nptst.assert_almost_equal(
            basis.evalDerivative(.67432, 3),
            sp.diff(basis_sp, s, 3).subs(s, .67432))

    def test_inner_product_00(self):
        basis_1 = StandardBasisMember(0)
        basis_2 = StandardBasisMember(0)

        nptst.assert_equal(
            basis_1.evalInnerProduct(basis_2, 0),
            basis_2.evalInnerProduct(basis_1, 0))

        nptst.assert_equal(
            basis_1.evalInnerProduct(basis_2, 0),
            1.0)

        nptst.assert_equal(
            basis_1.evalInnerProduct(basis_1, 0),
            1.0)

        nptst.assert_equal(
            basis_1.evalInnerProduct(basis_2, 1),
            basis_2.evalInnerProduct(basis_1, 1))

        nptst.assert_equal(
            basis_1.evalInnerProduct(basis_2, 1),
            0.0)

        nptst.assert_equal(
            basis_1.evalInnerProduct(basis_1, 1),
            0.0)

    def test_inner_product_with_self(self):

        for deg1 in range(15):
            basis_1 = StandardBasisMember(deg1)
            basis_2 = StandardBasisMember(deg1)

            for k in range(deg1 + 1):
                nptst.assert_equal(
                    basis_1.evalInnerProduct(basis_1, k),
                    basis_2.evalInnerProduct(basis_1, k),
                    )

                nptst.assert_almost_equal(
                    basis_1.evalInnerProduct(basis_1, k),
                    TestStandardBasisMember.symbolicInnerProduct(
                        deg1, deg1, k)
                    )

    def test_inner_product(self):

        for deg1 in range(15):
            for deg2 in range(15):
                basis_1 = StandardBasisMember(deg1)
                basis_2 = StandardBasisMember(deg2)

                for k in range(max(deg1, deg2) + 1):
                    nptst.assert_equal(
                        basis_1.evalInnerProduct(basis_2, k),
                        basis_2.evalInnerProduct(basis_1, k),
                        )

                    nptst.assert_almost_equal(
                        basis_1.evalInnerProduct(basis_2, k),
                        TestStandardBasisMember.symbolicInnerProduct(
                            deg1, deg2, k)
                        )

    @staticmethod
    def symbolicInnerProduct(deg1, deg2, k):
        s = sp.symbols('s')
        basis_1_sp = s**deg1 / np.math.factorial(deg1)
        basis_2_sp = s**deg2 / np.math.factorial(deg2)

        return sp.integrate(
            sp.diff(basis_1_sp, s, k)
            * sp.diff(basis_2_sp, s, k),
            (s, 0, 1)
        ).evalf()

    def test_generate_basis_vector(self):

        res = StandardBasisMember.generate_basis_vector(5)

        self.assertEqual(len(res), 5)
        self.assertEqual(res[0].degree, 0)
        self.assertEqual(res[1].degree, 1)
        self.assertEqual(res[2].degree, 2)
        self.assertEqual(res[3].degree, 3)
        self.assertEqual(res[4].degree, 4)

    def test_eval_basis_vector_l4_d1(self):
        basis_vector_l4_d1 = [
            StandardBasisMember(1),
            StandardBasisMember(1),
            StandardBasisMember(1),
            StandardBasisMember(1),
        ]

        res = StandardBasisMember.eval_basis_vector(
            basis_vector_l4_d1, 0.0, 0)
        nptst.assert_array_almost_equal(
            res, 0. * np.ones(4))

        res = StandardBasisMember.eval_basis_vector(
            basis_vector_l4_d1, .5, 0)
        nptst.assert_array_almost_equal(
            res, .5 * np.ones(4))

        res = StandardBasisMember.eval_basis_vector(
            basis_vector_l4_d1, 1., 0)
        nptst.assert_array_almost_equal(
            res, 1. * np.ones(4))

        res = StandardBasisMember.eval_basis_vector(
            basis_vector_l4_d1, 0., 1)
        nptst.assert_array_almost_equal(
            res, np.ones(4))

        res = StandardBasisMember.eval_basis_vector(
            basis_vector_l4_d1, .5, 1)
        nptst.assert_array_almost_equal(
            res, np.ones(4))

        res = StandardBasisMember.eval_basis_vector(
            basis_vector_l4_d1, 0., 7)
        nptst.assert_array_almost_equal(
            res, np.zeros(4))

        res = StandardBasisMember.eval_basis_vector(
            basis_vector_l4_d1, .321, 6)
        nptst.assert_array_almost_equal(
            res, np.zeros(4))


if __name__ == "__main__":
    unittest.main()
