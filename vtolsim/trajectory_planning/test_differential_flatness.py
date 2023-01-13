import unittest
import numpy as np
import numpy.testing as nptst
import sys
sys.path.append('..')

from trajectory_planning.differential_flatness import DifferentialFlatness, _project, _deSkew
from trajectory_planning.trajectory import Trajectory, XYZSinusoid

class TestUnitCircle(unittest.TestCase):
    
    def setUp(self):
        # unit circle 
        traj = XYZSinusoid(1, 1, 0, 1, 1, 0, 0, np.pi/2, 0)        
        self.dflat = DifferentialFlatness(traj)

    def test_position_0(self):
        nptst.assert_allclose(self.dflat.desiredPosition(0), np.array([[0., 1., 0.]]).T)

    def test_projection_matrix(self):
        e3 = np.array([[0, 0, 1]]).T
        proj_mat = _project(e3)
        nptst.assert_allclose(proj_mat, np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]]))

    def test_project_vector(self):
        e3 = np.array([[0, 0, 1]]).T
        proj_mat = _project(e3)

        # v is in the plane orthogonal to e3
        v = np.array([[0, 1, 0]]).T
        v_proj = proj_mat @ v
        nptst.assert_allclose(v_proj, v)

    def test_curvature_norm(self):
        # unit circle should have a curvature of 1 for all time
        nptst.assert_almost_equal(np.linalg.norm(self.dflat._curvature(0)), 2*np.pi)

        curv_1 = np.linalg.norm(self.dflat._curvature(.3))
        curv_2 = np.linalg.norm(self.dflat._curvature(.723))
        nptst.assert_allclose(curv_1, curv_2)
    
    def test_curvature_vec(self):
        nptst.assert_array_almost_equal(self.dflat._curvature(0), np.array([[0., -2*np.pi, 0.]]).T)

    def test_psi(self):
        nptst.assert_almost_equal(self.dflat._desiredPsi(0), 0.)
        nptst.assert_almost_equal(self.dflat._desiredPsi(.5), -np.pi)
        nptst.assert_almost_equal(self.dflat._desiredPsi(.25), -np.pi/2)

    def test_phi(self):
        # unit circle is very tight and fast, so phi should be high
        self.assertTrue(self.dflat._desiredPhi(0.0) <= np.pi/2)
        self.assertTrue(self.dflat._desiredPhi(0.0) >= 1.0)

    def test_theta(self):
        nptst.assert_almost_equal(self.dflat._desiredTheta(0), 0.)


    def test_desiredEuler(self):
        nptst.assert_array_almost_equal(self.dflat.desiredEuler(0.0), np.array([[1.32, 0.0, 0.0]]).T, decimal=2)

    def test_desiredQuat(self):
        nptst.assert_almost_equal(np.linalg.norm(self.dflat.desiredQuat(0.0)), 1.0)

    def test_desiredRotation(self):
        Rot = self.dflat.desiredRotation(0.0)
        nptst.assert_almost_equal(np.linalg.norm(Rot[0,:]), 1.0)
        nptst.assert_almost_equal(np.linalg.norm(Rot[1,:]), 1.0)
        nptst.assert_almost_equal(np.linalg.norm(Rot[2,:]), 1.0)

    def test_deSkew(self):
        skew = np.array([ [ 0, -3,  2], 
                          [ 3,  0, -1],
                          [-2,  1,  0]])
        ds = _deSkew(skew)
        nptst.assert_array_equal(ds, np.array([[1, 2, 3]]).T)

    def test_desiredRates(self):
        omega1 = self.dflat.desiredRates(0.0)
        omega2 = self.dflat.desiredRates(0.45)
        nptst.assert_equal(omega1.size, 3)
        nptst.assert_array_almost_equal(omega1, omega2)

    def test_desiredFromX(self):
        x = np.array([[1., 0., 0., 0., 0., 0., 1., 0., 0., 0.]]).T
        nearest = self.dflat.desiredState_fromX(x)
        nptst.assert_array_almost_equal(x[0:3], nearest[0:3], 1)

if __name__ == "__main__":
    unittest.main()


