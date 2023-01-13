"""
tools for working with quaternions
all quaternions should be column vectors (shape 4x1) in hamiltonian notation (real part first)
"""

import numpy as np

def q_circleTimes(p, q):
    """
    multiply two quaternions
    """
    pw = p.item(0)
    px = p.item(1)
    py = p.item(2)
    pz = p.item(3)
    qw = q.item(0)
    qx = q.item(1)
    qy = q.item(2)
    qz = q.item(3)

    return np.array([\
            [pw*qw - px*qx - py*qy - pz*qz], \
            [pw*qx + px*qw + py*qz - pz*qy], \
            [pw*qy - px*qz + py*qw + pz*qx], \
            [pw*qz + px*qy - py*qx + pz*qw]])

def q_exp(u):
    """
    quaternion exponential map (R3 -> S3)
    """
    u_norm = np.linalg.norm(u)
    if(np.abs(u_norm) <= 0.001):
        return q_exp_sa(u)
    u0 = u.item(0)/u_norm
    u1 = u.item(1)/u_norm
    u2 = u.item(2)/u_norm
    usin = np.sin(u_norm/2)

    return np.array([ \
        [np.cos(u_norm/2)], \
        [u0*usin], \
        [u1*usin], \
        [u2*usin]])


def q_exp_sa(u):
    """
    small angle quaternion exponential map (R3 -> S3)
    """
    u0 = u.item(0)
    u1 = u.item(1)
    u2 = u.item(2)
    return np.array([ \
        [1], \
        [u0/2], \
        [u1/2], \
        [u2/2]])

def q_boxPlus(q, u):
    """
    Add a quaternion and a vector (S3 X R3 -> S3)
    """
    u_quat = q_exp(u)
    return q_circleTimes(q, u_quat)

def q_boxMinus(p, q):
    """
    subtract two quaternions (S3 X S3 -> R3)
    p - q
    """
    dq = q_circleTimes(q_inv(q), p)

    # keep the quaternion always positive
    if (dq.item(0) < 0.0):
        dq *= -1.0

    return q_log(dq)

def q_conj(q):
    """
    conjugate of a quaternion
    """
    return np.array([\
            [q.item(0)],
            [-q.item(1)],
            [-q.item(2)],
            [-q.item(3)]])

def q_inv(q):
    """
    inverse of a quaternion
    """
    norm_q = np.linalg.norm(q)
    return q_conj(q)/(norm_q**2)

def q_log(q):
    """
    logarithmic map of a quaternion (S3 -> R3)
    """
    q_bar = q[1:]
    norm_q_bar = (np.linalg.norm(q_bar))
    q0 = q.item(0)
    if (np.abs(norm_q_bar) <= 0.001):
        return q_log_sa(q)
    return 2*np.arctan2(norm_q_bar, q0)*q_bar/norm_q_bar

def q_log_sa(q):
    """
    small angle approximation of the quaternion logarithmic map (S3 -> R3)
    """
    q_bar = q[1:]
    q0 = q.item(0)
    return 2*np.sign(q0)*q_bar

def q_rotate(v, q):
    """
    rotate the vector v by the quaternion q
    """
    v_aug     = np.array([[0, v.item(0), v.item(1), v.item(2)]]).T
    rot = q_circleTimes(q, q_circleTimes(v_aug, q_conj(q)))
    return rot[1:]


def state_boxPlus(x1, x2):
    """ 
    Compute x1 [+] x2 using the appropriate rules
    R3xR3xS3 X R3xR3xR3 -> R3XR3XS3
    """
    ret = np.zeros((10,1))
    ret[:6] = x1[:6] + x2[:6]
    ret[6:10] = q_boxPlus(x1[6:10], x2[6:9])

    return ret


def state_boxMinus(x1, x2):
    """
    Compute x1 [-] x2 using the appropriate subtraction rules
    R3xR3xS3 X R3xR3xS3 -> R3XR3XR3
    """
    ret = np.zeros((9,1))
    ret[:6] = x1[:6] - x2[:6]
    ret[6:9] = q_boxMinus(x1[6:10], x2[6:10])

    return ret
