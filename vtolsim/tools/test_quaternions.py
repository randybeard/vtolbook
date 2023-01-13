#! /usr/bin/env python3
import unittest
import numpy as np
import numpy.testing as nptst

from quaternions import *

class TestQuat(unittest.TestCase):

    def test_circleTimes_ident(self):
        p = np.array([[1, 0, 0, 0]]).T
        q = np.array([[1, 0, 0, 0]]).T

        res = q_circleTimes(p, q)
        nptst.assert_array_almost_equal(res, q)
        nptst.assert_almost_equal(np.linalg.norm(res), 1.0)

    def test_circleTimes_ident_2(self):
        p = np.array([[1, 0, 0, 0]]).T
        q = np.array([[.5, .5, .5, .5]]).T

        res = q_circleTimes(p, q)
        nptst.assert_array_almost_equal(res, q)
        nptst.assert_almost_equal(np.linalg.norm(res), 1.0)
    
    def test_q_exp(self):
        u = np.array([[1, 1, 1]]).T
        res = q_exp(u)
        nptst.assert_equal(res.size, 4)
        nptst.assert_almost_equal(np.linalg.norm(res), 1.0)

    def test_q_exp_sa(self):
        u = np.array([[.00001, 0, 0]]).T
        res = q_exp(u)
        nptst.assert_equal(res.size, 4)
        nptst.assert_almost_equal(np.linalg.norm(res), 1.0)

    def test_q_log_sa(self):
        u = np.array([[1, .00001, 0, 0]]).T
        res = q_log(u)
        nptst.assert_equal(res.size, 3)

    def test_q_boxPlus(self):
        u = np.array([[1, 0, 0]]).T
        q = np.array([[1, 0, 0, 0]]).T
        res = q_boxPlus(q, u)
        nptst.assert_equal(res.size, 4)
        nptst.assert_almost_equal(np.linalg.norm(res), 1.0)
        self.assertFalse(np.array_equal(q, res))

    def test_q_boxMinus(self):
        p = np.array([[0.57735027], [0.57735027], [0.], [0.57735027]])       
        q = np.array([[.5, .5, .5, .5]]).T
        res = q_boxMinus(p, q)
        nptst.assert_equal(res.size, 3)
    
    def test_q_exp_log(self):
        u = np.array([[1, 1, 1]]).T
        q_res = q_exp(u)
        u_res = q_log(q_res)
        nptst.assert_equal(u_res.size, 3)
        nptst.assert_array_almost_equal(u, u_res)
        
    def test_q_log_exp(self):
        q = np.array([[0.57735027], [0.57735027], [0.], [0.57735027]])       
        u_res = q_log(q)
        q_res = q_exp(u_res)
        nptst.assert_equal(q_res.size, 4)
        nptst.assert_array_almost_equal(q, q_res)
    
if __name__ == "__main__":
    unittest.main()

