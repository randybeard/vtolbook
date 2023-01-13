import numpy as np


def quat2euler(quat):
    """Get euler angle representation as a 3d vector
    returns [roll, pitch, yaw]
    """
    q0, qx, qy, qz = quat
    roll = np.arctan2(2.0*(q0*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = np.arcsin(2.0*(q0*qy - qz*qx))
    yaw = np.arctan2(2.0*(q0*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return np.array([roll, pitch, yaw])


def velocities_to_headings(list_vel_vec, current_yaw):
    """
    Takes a list of inertial velocities (3 vectors)
    and converts to inertial heading angles.

    :param list_vel_vec: List of velocity 3 vectors
    :return: List of desired heading values
             corresponding to the velocity vectors
    :rtype: list
    """
    yaw_d = []
    yaw_prev = current_yaw
    alpha = .5
    for v in list_vel_vec:
        if np.abs(v[0]) > 1.0 or np.abs(v[1]) > 1.0:
            yaw_new = np.arctan2(v[1], v[0])
            yaw_new = (alpha)*yaw_new + (1-alpha)*yaw_prev
            yaw_d.append(yaw_new)
            yaw_prev = yaw_new
        else:
            yaw_d.append(yaw_prev)
    return yaw_d
