#! /usr/bin/env python3
import unittest
import numpy as np
import numpy.testing as nptst
import sys
sys.path.append('..')

from trajectory_planning.differential_flatness import DifferentialFlatness 
from trajectory_planning.trajectory import XYZSinusoid
from lqr_dynamics import *

class TestLQRDynamics(unittest.TestCase):
    def setUp(self):

        # 100m circle
        traj = XYZSinusoid(100, 100, 0, 1, 1, 0, 0, np.pi/2, 0)        
        # create mock differential flatness
        self.df_traj = DifferentialFlatness(traj)


    def test_mixer_45(self):
        F = np.array([.5, .5])
        res = mixer(F)
        nptst.assert_almost_equal(res.servo_right, np.radians(45), 1)
        nptst.assert_almost_equal(res.servo_left, np.radians(45), 1)

        self.assertTrue(res.throttle_rear > 0.0)
        self.assertTrue(res.throttle_rear < 1.0)

        self.assertTrue(res.throttle_right > 0.0)
        self.assertTrue(res.throttle_right < 1.0)

        self.assertTrue(res.throttle_left > 0.0)
        self.assertTrue(res.throttle_left < 1.0)

    def test_mixer_0(self):
        F = np.array([1, 0])
        res = mixer(F)
        nptst.assert_almost_equal(res.servo_right, np.radians(0))
        nptst.assert_almost_equal(res.servo_left, np.radians(0))

        nptst.assert_almost_equal(res.throttle_rear, 0.0)
        nptst.assert_almost_equal(res.throttle_right, 1.0)
        nptst.assert_almost_equal(res.throttle_left, 1.0)

    def test_mixer_90(self):
        F = np.array([0, 1])
        res = mixer(F)
        nptst.assert_almost_equal(res.servo_right, np.radians(90))
        nptst.assert_almost_equal(res.servo_left, np.radians(90))

        nptst.assert_almost_equal(res.throttle_rear, 1.0, 1)
        nptst.assert_almost_equal(res.throttle_right, 1.0, 1)
        nptst.assert_almost_equal(res.throttle_left, 1.0, 1)

    def test_f(self):
        x = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]).T
        u = np.array([[0, .7, 0, 0, 0]]).T
        res = f(x, u)

        nptst.assert_equal(res.size, 10)
        nptst.assert_array_almost_equal(res[6:10, 0], np.array([0, 0, 0, 0]))
        self.assertTrue(np.linalg.norm(res[3:6, 0]) > 0.0)
    
    def test_df_dx(self):
        x_tilde = np.array([[0., 0., 0., 0., 0., 0., 0., 0., 0.]]).T
        x_des   = np.array([[10, 5, -10, 20, 0, 5, 1, 0, 0, 0]]).T

        u_tilde = np.array([[0.1, .03, .1, 0.1, 0]]).T
        u_des   = np.array([[.2, 1, 1, 0, 0]]).T

        res = df_dx(x_tilde, x_des, u_tilde, u_des, self.df_traj)

        nptst.assert_equal(res.size, (x_tilde.size**2))
        print("\ndf_dx\n")
        print(res, "\n")

    def test_df_du(self):
        x_tilde = np.array([[0., 0., 0., 0., 0., 0., 0., 0., 0.]]).T
        x_des   = np.array([[10, 5, -10, 20, 0, 5, 1, 0, 0, 0]]).T

        u_tilde = np.array([[0.1, .03, .1, 0.1, 0]]).T
        u_des   = np.array([[.2, 1, 1, 0, 0]]).T

        res = df_du(x_tilde, x_des, u_tilde, u_des, self.df_traj)

        nptst.assert_equal(res.size, (x_tilde.size*u_tilde.size))
        print("\ndf_du\n")
        print(res, "\n")

if __name__ == "__main__":
    unittest.main()

