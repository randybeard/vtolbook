#! /usr/bin/env python3
import unittest
import numpy as np
import numpy.testing as nptst

from rotations import *

class TestRotations(unittest.TestCase):

    def test_random_rotation(self):
        # test 100 different random rotation matrices
        np.random.seed(0) # make it repeatable
        for _ in range(100):
            R = random_rotation()
            # ensure R is orthogonal
            np.testing.assert_array_almost_equal(R.T@R, np.eye(3))
            # ensure R has determinant 1
            np.testing.assert_almost_equal(np.linalg.det(R), 1.0)

    def test_rotation2quaternion(self):
        R1 = np.eye(3)
        q1 = Rotation2Quaternion(R1)
        np.testing.assert_almost_equal(q1.item(0), 1.0)
        np.testing.assert_almost_equal(q1.item(1), 0.0)
        np.testing.assert_almost_equal(q1.item(2), 0.0)
        np.testing.assert_almost_equal(q1.item(3), 0.0)

        # test 100 different random rotation matrices
        np.random.seed(123) # make it repeatable
        for _ in range(100):
            R = random_rotation()
            q = Rotation2Quaternion(R)
            np.testing.assert_almost_equal(np.linalg.norm(q), 1.0)

    def test_quaternion2rotation(self):
        q1 = np.array([1., 0., 0., 0.]).reshape(-1, 1)
        R1 = Quaternion2Rotation(q1)
        np.testing.assert_almost_equal(R1, np.eye(3))

        # test 100 different random quaternions
        np.random.seed(1) # make it repeatable
        for _ in range(100):
            q = random_quaternion()
            R = Quaternion2Rotation(q)
            # ensure R is orthogonal
            np.testing.assert_array_almost_equal(R.T@R, np.eye(3))
            # ensure R has determinant 1
            np.testing.assert_almost_equal(np.linalg.det(R), 1.0)

    def test_rotation2rotation_via_quaternion(self):
        # test 100 different random rotation matrices
        np.random.seed(777) # make it repeatable
        for _ in range(100):
            R1 = random_rotation()
            q = Rotation2Quaternion(R1)
            R2 = Quaternion2Rotation(q)

            # ensure R2 in SO3
            np.testing.assert_array_almost_equal(R2.T@R2, np.eye(3))
            np.testing.assert_almost_equal(np.linalg.det(R2), 1.0)

            # ensure R1 = R2
            np.testing.assert_array_almost_equal(R1, R2)

    def test_rotation2rotation_via_euler(self):
        # test 100 different random rotation matrices
        np.random.seed(777) # make it repeatable
        for _ in range(100):
            R1 = random_rotation()
            phi, theta, psi = Rotation2Euler(R1)
            R2 = Euler2Rotation(phi, theta, psi)

            # ensure R2 in SO3
            np.testing.assert_array_almost_equal(R2.T@R2, np.eye(3))
            np.testing.assert_almost_equal(np.linalg.det(R2), 1.0)

            # ensure R1 = R2
            np.testing.assert_array_almost_equal(R1, R2)

    def test_quaternion2quaternion_via_rotation(self):
        # test 100 different random quaternions
        np.random.seed(321) # make it repeatable
        for _ in range(100):
            q1 = random_quaternion()

            R = Quaternion2Rotation(q1)
            q2 = Rotation2Quaternion(R).reshape(-1)

            # ensure q2 in S3
            np.testing.assert_almost_equal(np.linalg.norm(q2), 1.0)

            # ensure q1 = q2
            np.testing.assert_array_almost_equal(q1, q2)

    def test_quaternion2quaternion_via_euler(self):
        # test 100 different random quaternions
        np.random.seed(321) # make it repeatable
        for _ in range(100):
            q1 = random_quaternion().reshape(-1, 1)

            phi, theta, psi = Quaternion2Euler(q1)
            q2 = Euler2Quaternion(phi, theta, psi)

            # ensure q2 in S3
            np.testing.assert_almost_equal(np.linalg.norm(q2), 1.0)
            # ensure q1 = q2
            np.testing.assert_array_almost_equal(q1, q2)

def random_quaternion():
    q_dir = 2.*np.random.rand(4) - 1.
    q = q_dir/np.linalg.norm(q_dir)
    if q[0] < 0:
        q *= -1.
    return q

def random_rotation():
    """
    Generate a random rotation matrix
    """
    # Random vectors with components in [-1, 1]
    random_vector_1 = 2.*np.random.rand(3) - 1.
    random_vector_2 = 2.*np.random.rand(3) - 1.

    random_vector_1_normed = random_vector_1/np.linalg.norm(random_vector_1)
    random_vector_2_normed = random_vector_2/np.linalg.norm(random_vector_2)

    Rx = random_vector_1_normed
    Ry_dir = np.cross(Rx, random_vector_2_normed)
    Ry = Ry_dir/np.linalg.norm(Ry_dir)
    Rz = np.cross(Rx, Ry)

    R = np.column_stack([Rx, Ry, Rz])
    return R
            

if __name__ == "__main__":
    unittest.main()