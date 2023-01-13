"""
various tools to be used in quadrotorsim
"""
from numpy import array, sin, cos, sinc, arctan2, arcsin, arccos, trace, sqrt, eye, sign
from numpy.linalg import norm, det


def quaternion_to_euler(quaternion):
    """
    converts a quaternion attitude to an euler angle attitude
    :param quaternion: the quaternion to be converted to euler angles
    :return: the euler angle equivalent (phi, theta, psi) in a array
    """
    q0 = quaternion.item(0)
    qx = quaternion.item(1)
    qy = quaternion.item(2)
    qz = quaternion.item(3)
    phi = arctan2(2.0 * (qy * qz + q0 * qx), q0**2.0 - qx*2.0 - qy**2.0 + qz**2.0 )
    theta = arcsin(2.0 * (q0 * qy - qx * qz))
    psi = arctan2(2.0 * (qx * qy + q0 * qz), q0**2.0 + qx**2.0 - qy**2.0 - qz**2.0)
    return phi, theta, psi


def euler_to_quaternion(phi=0., theta=0., psi=0.):
    """
    Converts an euler angle attitude to a quaternian attitude
    :return: Quaternian attitude in array(e0, e1, e2, e3)
    """

    e0 = cos(psi/2.0) * cos(theta/2.0) * cos(phi/2.0) + sin(psi/2.0) * sin(theta/2.0) * sin(phi/2.0)
    e1 = cos(psi/2.0) * cos(theta/2.0) * sin(phi/2.0) - sin(psi/2.0) * sin(theta/2.0) * cos(phi/2.0)
    e2 = cos(psi/2.0) * sin(theta/2.0) * cos(phi/2.0) + sin(psi/2.0) * cos(theta/2.0) * sin(phi/2.0)
    e3 = sin(psi/2.0) * cos(theta/2.0) * cos(phi/2.0) - cos(psi/2.0) * sin(theta/2.0) * sin(phi/2.0)

    return array([[e0],[e1],[e2],[e3]])


def euler_to_rotation(phi=0, theta=0, psi=0):
    """
    Converts euler angles to rotation matrix (R_b^i)
    """
    c_phi = cos(phi)
    s_phi = sin(phi)
    c_theta = cos(theta)
    s_theta = sin(theta)
    c_psi = cos(psi)
    s_psi = sin(psi)

    R_roll = array([[1, 0, 0],
                       [0, c_phi, -s_phi],
                       [0, s_phi, c_phi]])
    R_pitch = array([[c_theta, 0, s_theta],
                        [0, 1, 0],
                        [-s_theta, 0, c_theta]])
    R_yaw = array([[c_psi, -s_psi, 0],
                      [s_psi, c_psi, 0],
                      [0, 0, 1]])
    R = R_yaw @ R_pitch @ R_roll
    return R


def quaternion_to_rotation(quaternion):
    """
    converts a quaternion attitude to a rotation matrix
    """
    e0 = quaternion.item(0)
    e1 = quaternion.item(1)
    e2 = quaternion.item(2)
    e3 = quaternion.item(3)

    R = array([[e1 ** 2.0 + e0 ** 2.0 - e2 ** 2.0 - e3 ** 2.0, 2.0 * (e1 * e2 - e3 * e0), 2.0 * (e1 * e3 + e2 * e0)],
                  [2.0 * (e1 * e2 + e3 * e0), e2 ** 2.0 + e0 ** 2.0 - e1 ** 2.0 - e3 ** 2.0, 2.0 * (e2 * e3 - e1 * e0)],
                  [2.0 * (e1 * e3 - e2 * e0), 2.0 * (e2 * e3 + e1 * e0), e3 ** 2.0 + e0 ** 2.0 - e1 ** 2.0 - e2 ** 2.0]])
    R = R/det(R)
    return R


def rotation_to_quaternion(R):
    """
    converts a rotation matrix to a unit quaternion
    """
    r11 = R[0][0]
    r12 = R[0][1]
    r13 = R[0][2]
    r21 = R[1][0]
    r22 = R[1][1]
    r23 = R[1][2]
    r31 = R[2][0]
    r32 = R[2][1]
    r33 = R[2][2]

    tmp0=r11+r22+r33
    if tmp0>0:
        q0 = 0.5*sqrt(1+tmp0)
    else:
        q0 = 0.5*sqrt(((r12-r21)**2+(r13-r31)**2+(r23-r32)**2)/(3-tmp0))

    tmpx=r11-r22-r33
    if tmpx>0:
        qx = 0.5*sqrt(1+tmpx)
    else:
        qx = 0.5*sqrt(((r12+r21)**2+(r13+r31)**2+(r23-r32)**2)/(3-tmpx))
    qx = sign(r32-r23) * qx

    tmpy=-r11+r22-r33
    if tmpy>0:
        qy = 0.5*sqrt(1+tmpy)
    else:
        qy = 0.5*sqrt(((r12+r21)**2+(r13-r31)**2+(r23+r32)**2)/(3-tmpy))
    qy = sign(r13-r31) * qy

    tmpz=-r11+-22+r33
    if tmpz>0:
        qz = 0.5*sqrt(1+tmpz)
    else:
        qz = 0.5*sqrt(((r12-r21)**2+(r13+r31)**2+(r23+r32)**2)/(3-tmpz))
    qz = sign(r21-r12) * qz

    return array([[q0], [qx], [qy], [qz]])

def rotation_to_euler(R):
    """
    converts a rotation matrix to euler angles
    """
    quat = rotation_to_quaternion(R)
    phi, theta, psi = quaternion_to_euler(quat)
    return phi, theta, psi


def hat(omega):
    """
    vector to skew symmetric matrix associated with cross product
    """
    a = omega.item(0)
    b = omega.item(1)
    c = omega.item(2)
    omega_hat = array([[0, -c, b],
                        [c, 0, -a],
                        [-b, a, 0]])
    return omega_hat

def vee(M):
    """
    Maps skew-symmetric matrix to a vector
    """
    if norm(M+M.T) != 0:
        print("M is not skew-symmetric")
        m = float("nan")
    else:
        m = array([[M[2][1]], [-M[2][0]], [M[1][0]]])
    return m



def quat_hat(omega):
    """
    vector to skew symmetric matrix associated with cross product-quaternion
    """
    p = omega.item(0)
    q = omega.item(1)
    r = omega.item(2)
    omega_hat = array([[0, -p, -q, -r],
                          [p, 0, r, -q],
                          [q, -r, 0, p],
                          [r, q, -p, 0]])
    return omega_hat

def logR(R):
    """
    Log of a rotation matrix
    """
    tmp1 = sat((trace(R) - 1) / 2, +1, -1)
    theta = arccos(tmp1)
    tmp = sinc(theta)
    log_of_R = 0.5 * (R - R.T) / tmp
    # if tmp != 0:
    #     log_of_R = 0.5 * ( R - R.T ) / tmp
    # else:
    #     print("log of R is not defined")
    #     log_of_R = float("nan")
    return log_of_R

def leftJacobian(r):
    """
    the left Jacobian of the rotation vector r
    """
    phi = norm(r)
    u = r/phi
    J = eye(3) \
        + sin(phi/2) * sinc(phi/2) * hat(u) \
        + (1 - sinc(phi)) * hat(u) @ hat(u)
    return J

def leftJacobianInv(r):
    """
    the inverse of the left Jacobian of rotation vector r
    """
    phi = norm(r)
    if phi == 0:
        u = 0 * r
    else:
        u = r/phi
    Jinv = eye(3) \
           - phi/2 * hat(u) \
           + (1 - cos(phi/2) / sinc(phi/2)) * hat(u) @ hat(u)
    return Jinv

def sat(x, up, low):
    if x > up:
        y=up
    elif x < low:
        y = low
    else:
        y = x
    return y



