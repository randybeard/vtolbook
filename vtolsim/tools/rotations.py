"""
various tools to be used in mavPySim
"""
import numpy as np
import scipy.linalg as linalg

def Quaternion2Euler(quaternion):
    """
    converts a quaternion attitude to an euler angle attitude
    :param quaternion: the quaternion to be converted to euler angles in a np.matrix
    :return: the euler angle equivalent (phi, theta, psi) in a np.array
    """
    e0 = quaternion.item(0)
    e1 = quaternion.item(1)
    e2 = quaternion.item(2)
    e3 = quaternion.item(3)
    phi = np.arctan2(2.0 * (e0 * e1 + e2 * e3), e0**2.0 + e3**2.0 - e1**2.0 - e2**2.0)
    theta = np.arcsin(2.0 * (e0 * e2 - e1 * e3))
    psi = np.arctan2(2.0 * (e0 * e3 + e1 * e2), e0**2.0 + e1**2.0 - e2**2.0 - e3**2.0)

    return phi, theta, psi

def Euler2Quaternion(phi, theta, psi):
    """
    Converts an euler angle attitude to a quaternian attitude
    :param euler: Euler angle attitude in a np.matrix(phi, theta, psi)
    :return: Quaternian attitude in np.array(e0, e1, e2, e3)
    """

    e0 = np.cos(psi/2.0) * np.cos(theta/2.0) * np.cos(phi/2.0) + np.sin(psi/2.0) * np.sin(theta/2.0) * np.sin(phi/2.0)
    e1 = np.cos(psi/2.0) * np.cos(theta/2.0) * np.sin(phi/2.0) - np.sin(psi/2.0) * np.sin(theta/2.0) * np.cos(phi/2.0)
    e2 = np.cos(psi/2.0) * np.sin(theta/2.0) * np.cos(phi/2.0) + np.sin(psi/2.0) * np.cos(theta/2.0) * np.sin(phi/2.0)
    e3 = np.sin(psi/2.0) * np.cos(theta/2.0) * np.cos(phi/2.0) - np.cos(psi/2.0) * np.sin(theta/2.0) * np.sin(phi/2.0)

    # ensure sign convention is kept
    quat =  np.array([[e0],[e1],[e2],[e3]])
    if quat.item(0) < 0:
        quat = -1.*quat

    return quat

def Euler2Rotation(phi, theta, psi):
    """
    Converts euler angles to rotation matrix (R_b^i)
    """
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    R_roll = np.array([[1, 0, 0],
                       [0, c_phi, -s_phi],
                       [0, s_phi, c_phi]])
    R_pitch = np.array([[c_theta, 0, s_theta],
                        [0, 1, 0],
                        [-s_theta, 0, c_theta]])
    R_yaw = np.array([[c_psi, -s_psi, 0],
                      [s_psi, c_psi, 0],
                      [0, 0, 1]])
    #R = np.dot(R_yaw, np.dot(R_pitch, R_roll))
    R = R_yaw @ R_pitch @ R_roll

    # rotation is body to inertial frame
    # R = np.array([[c_theta*c_psi, s_phi*s_theta*c_psi-c_phi*s_psi, c_phi*s_theta*c_psi+s_phi*s_psi],
    #               [c_theta*s_psi, s_phi*s_theta*s_psi+c_phi*c_psi, c_phi*s_theta*s_psi-s_phi*c_psi],
    #               [-s_theta, s_phi*c_theta, c_phi*c_theta]])

    return R

def Quaternion2Rotation(quaternion):
    q0 = quaternion.item(0)
    qbar = quaternion[1:].reshape(-1)

    R = np.eye(3) + 2*q0*hat(qbar) + 2*(hat(qbar)@hat(qbar))

    return R

def Rotation2Quaternion(R):
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

    tmp=r11+r22+r33
    if tmp>0:
        e0 = 0.5*np.sqrt(1+tmp)
    else:
        e0 = 0.5*np.sqrt(((r12-r21)**2+(r13-r31)**2+(r23-r32)**2)/(3-tmp))

    tmp=r11-r22-r33
    if tmp>0:
        e1 = 0.5*np.sqrt(1+tmp)
    else:
        e1 = 0.5*np.sqrt(((r12+r21)**2+(r13+r31)**2+(r23-r32)**2)/(3-tmp))

    tmp=-r11+r22-r33
    if tmp>0:
        e2 = 0.5*np.sqrt(1+tmp)
    else:
        e2 = 0.5*np.sqrt(((r12+r21)**2+(r13-r31)**2+(r23+r32)**2)/(3-tmp))

    tmp=-r11-r22+r33
    if tmp>0:
        e3 = 0.5*np.sqrt(1+tmp)
    else:
        e3 = 0.5*np.sqrt(((r12-r21)**2+(r13+r31)**2+(r23+r32)**2)/(3-tmp))

    # signs
    e1 = np.sign(r32-r23)*e1
    e2 = np.sign(r13-r31)*e2
    e3 = np.sign(r21-r12)*e3

    quat = np.array([[e0], [e1], [e2], [e3]])
    
    return quat

def Rotation2Euler(R):
    """
    Convert a body to inertial rotation matrix 
    to euler angles for plotting
    """
    # R = R.T 
    if np.abs(R[2,0]) <= (1.0 - 1e-8): # R31 != 1.0
        theta = -np.arcsin(R[2,0])
        ctheta = np.cos(theta)
        phi = np.arctan2(R[2,1]/ctheta, R[2,2]/ctheta)
        psi = np.arctan2(R[1,0]/ctheta, R[0,0]/ctheta)
    else:
        phi = 0
        if R[2,0] < 0:
            theta = np.pi/2
            psi = phi + np.arctan2(R[0,1], R[0,2])
        else:
            theta = -np.pi/2
            psi = -phi + np.arctan2(-R[0,1], -R[0,2])
    
    return phi, theta, psi # roll, pitch, yaw


def hat(omega):
    """
    vector to skew symmetric matrix associated with cross product
    """
    a = omega.item(0)
    b = omega.item(1)
    c = omega.item(2)

    omega_hat = np.array([[0, -c, b],
                          [c, 0, -a],
                          [-b, a, 0]])
    return omega_hat    

def vee(M):
    """
    Maps skew-symmetric matrix to a vector
    """
    [r,c] = M.shape
    # if square
    if r == c:
        if np.linalg.norm(M+M.T) != 0:
            print("M is not skew-symmetric")
            m = float("nan")
        else:
            m = np.array([[M[2][1]], [-M[2][0]], [M[1][0]]])
        return m
    else:
        return np.array([M[2,1], M[0,2], M[1,0]])

def logR(R):
    """
    Log of a rotation matrix
    """
    tmp1 = sat((np.trace(R) - 1) / 2, +1, -1)
    theta = np.arccos(tmp1)
    tmp = np.sinc(theta)
    log_of_R = 0.5 * (R - R.T) / tmp
    # if tmp != 0:
    #     log_of_R = 0.5 * ( R - R.T ) / tmp
    # else:
    #     print("log of R is not defined")
    #     log_of_R = float("nan")
    return log_of_R

def leftJacobianInv(r):
    """
    the inverse of the left Jacobian of rotation vector r
    """
    phi = np.linalg.norm(r)
    if phi == 0:
        u = 0 * r
    else:
        u = r/phi
    Jinv = np.eye(3) \
           - phi/2 * hat(u) \
           + (1 - np.cos(phi/2) / np.sinc(phi/2)) * hat(u) @ hat(u)
    return Jinv

def sat(x, up, low):
    if x > up:
        y=up
    elif x < low:
        y = low
    else:
        y = x
    return y